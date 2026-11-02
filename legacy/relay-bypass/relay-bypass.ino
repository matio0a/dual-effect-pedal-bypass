// This is v1.49 - for use with the v1.4 PCB.
// It adds memory for the most recent engage/bypass setting.
// It also disables the muting since we're sold out of optocouplers
// and they've been discontinued.

// Pulls in the AVR register definitions (DDRB, PORTB, PINB, PB0..PB4) for the selected chip.
#include <avr/io.h>
// Pulls in _delay_ms(), a busy-wait delay calibrated to the CPU clock (F_CPU).
#include <util/delay.h>
// Pulls in the EEPROM routines (eeprom_is_ready, eeprom_read_byte, eeprom_update_byte).
#include <avr/eeprom.h>

//------------------------------------------------
// BASIC BEHAVIOR SETTINGS:

// which uC
// If enabled, selects the ATtiny85 (512-byte EEPROM). Only affects EEPROM_SIZE below.
#define ATTINY85
// If enabled, selects the ATtiny13 (64-byte EEPROM). Only affects EEPROM_SIZE below.
// #define ATTINY13

// If enabled, read_switch() inverts the reading for a normally-closed footswitch.
//#define NORMALLY_CLOSED

// If you define SIMPLIFIED_DEBOUNCE it will trust the
// first instance of a button press, then ignore subsequent
// presses for the SIMPLIFIED_DEBOUNCE_DELAY.
// Otherwise if this is not defined it will do a more traditional
// debounce that looks for a stabilized change of state.
// #define SIMPLIFIED_DEBOUNCE

// Define DISABLE_TEMPORARY_SWITCH to turn off the ability to hold
// the switch to temporarily engage/bypass.
//#define DISABLE_TEMPORARY_SWITCH

// Default for the mute feature. Value greater than 0 enables the MUTE feature.
#define USE_MUTE 1
//------------------------------------------------



// If ATTINY85 is defined, use the next line, otherwise the #else branch.
#ifdef ATTINY85
// The ATtiny85 has 512 bytes of EEPROM.
# define EEPROM_SIZE 512
#else
// The ATtiny13 has 64 bytes of EEPROM.
# define EEPROM_SIZE 64
#endif

// Each saved bypass state takes one EEPROM byte.
#define EEPROM_CHUNK_SIZE 1 // we're choosing to use 1 byte
// Bit 7 marks the byte that holds the current (most recent) state - the wear-leveling "you are here" flag.
#define EEPROM_ADDR_FLAG_MASK 0b10000000  // chosen to indicate current addr
// EEPROM address 0 holds the power-on mode (ENGAGE / BYPASS / REMEMBER).
#define ADDR_BOOT_SETTING   0x0
// First address of the wear-leveling ring that stores the remembered bypass state.
#define ADDR_BYPASS_SETTING 0x1 // starting location to look
// RAM copy of the EEPROM address of the currently flagged state byte.
uint16_t eeprom_addr = ADDR_BYPASS_SETTING;

// These will also indicate the number of LED blinks
// Power-on mode code: always start engaged (effect on).
#define ON_BOOT_ENGAGE   0x1
// CLAUDE: Power-on mode code: always start bypassed (effect off).
#define ON_BOOT_BYPASS   0x2
// CLAUDE: Power-on mode code: restore whatever state was active at power-off.
#define ON_BOOT_REMEMBER 0x3

// Pick the debounce time to match the debounce method.
#ifdef SIMPLIFIED_DEBOUNCE
// Simple method: lockout time (ms) after an accepted switch edge.
# define DEBOUNCE_DELAY 250
#else
// The reading must stay stable this many ms before it is accepted.
# define DEBOUNCE_DELAY 14
#endif

// Toggle switch input on PB0 (physical pin 5). DRV = pulled to ground; RAT = pushed to +5V.
#define PIN_TOGGLE PB0
// Footswitch input on PB2 (physical pin 6). Pressed = pulled to ground.
#define PIN_FOOTSWITCH PB1
// Relay drive on PB2 (physical pin 7). Pulsed high to switch the relay to engaged.
#define PIN_ENGAGE PB2
// Relay drive on PB3 (physical pin 2). Pulsed high to switch the relay to RAT; pulsed low = switch relay to DRV. Default state is DRV.
#define PIN_SELECT PB3
// OPTO mute drive on PB4 (physical pin 3). Pulsed high = RAT muted, pulsed low = DRV muted, tri-stated = both RAT & DRV muted.
#define PIN_MUTE PB4
// Status LED output on PB5 (physical pin 1). High = effect engaged.
#define PIN_LED PB5


// If you hold down the switch for at least this number of ms, then
// we assume you are holding it down to temporarily toggle the state,
// and want it to toggle back on release.
// i.e. it's not a quick press and release.
// A press held longer than 500 ms is treated as momentary: the state flips back on release.
// #define TEMPORARY_SWITCH_TIME 500


/* 1. When switch is pressed, start muting for MUTE_LENGTH ms.
 * 2. Wait MUTED_RELAY_DELAY before actually toggling the relay though.
 */
// Intended mute time (ms) from when the mute pin goes high. The relay pulse adds to it (see toggle_bypass_state()).
#define MUTE_LENGTH 35
// Time (ms) between muting and switching the relay, so the mute is fully on first.
#define MUTED_RELAY_DELAY 20

// Length (ms) of the pulse sent to the latching relay.
// #define RELAY_SWITCH_TIME 40


// Current state: 1 = bypassed (effect off), 0 = engaged (effect on). setup() sets the real value.
uint8_t is_bypassed  = 0;
// Run-time copy of USE_MUTE. Non-zero makes toggle_bypass_state() drive the mute pin.
uint8_t use_mute     = USE_MUTE;
// Debounced switch state: 0 = pressed, non-zero = released.
uint8_t sw_state;     // its debounced state. i.e. what we assume is intended by player
// Raw switch reading from the previous pass of loop().
uint8_t sw_last_loop; // state last loop

// millis() timestamp for debouncing (last raw change, or last accepted edge with SIMPLIFIED_DEBOUNCE).
unsigned long sw_stable_since = 0;
// millis() timestamp of the last accepted press, used to tell a tap from a hold.
unsigned long sw_pressed_at   = 0;


// 1 = save every state change to EEPROM (REMEMBER mode only), 0 = don't write.
uint8_t use_eeprom = 0;

// Setup function runs once during power up. The user can enter the setup mode here by holding the footswitch pressed.
void setup() {
  // Clear the PB0 bit in the data-direction register, making the toggle switch an input.
  DDRB &= ~(1 << PIN_TOGGLE); // input
  // Set PB0 in PORTB while it is an input, enabling the internal pull-up.
  PORTB |= (1 << PIN_TOGGLE); // activate pull-up resistor
  // Clear the PB1 bit in the data-direction register, making the footswitch pin an input.
  DDRB &= ~(1 << PIN_FOOTSWITCH); // input
  // Set PB1 in PORTB while it is an input, enabling the internal pull-up so a released switch reads high.
  PORTB |= (1 << PIN_FOOTSWITCH); // activate pull-up resistor
  // CLAUDE: Make the relay engage pin an output (starts low after reset).
  DDRB |= (1 << PIN_ENGAGE); // output
  // CLAUDE: Make the effect selection relay pin an output (starts low after reset).
  DDRB |= (1 << PIN_SELECT); // output
  // CLAUDE: Make the mute pin an output (stays low unless muting is enabled).
  DDRB |= (1 << PIN_MUTE); // output
  // Make the LED pin an output (starts low after reset).
  DDRB |= (1 << PIN_LED); // output



  // Wait 5 ms for the pull-up to charge the switch line.
  _delay_ms(5);
  // First raw switch reading. 0 here means the switch is being held at power-up.
  sw_last_loop = read_switch();
  // Start the debounced state equal to the raw reading so no edge is detected straight away.
  sw_state = sw_last_loop;

  // LED blink count for confirming a power-on mode change. Stays 0 unless the mode is changed below.
  uint8_t blink_n_times = 0;

  // Only touch the EEPROM if it is not busy with a write.
  if (eeprom_is_ready()) {
    // Read the stored power-on mode from EEPROM address 0.
    uint8_t on_boot_setting = eeprom_read_byte((const uint8_t *)ADDR_BOOT_SETTING);

    // If the mode was set to "always engaged" 
    if (on_boot_setting == ON_BOOT_ENGAGE) {
      // Start with the effect on.
      is_bypassed = 0;

    // If the mode was set to "always bypassed"
    } else if (on_boot_setting == ON_BOOT_BYPASS) {
      // Start with the effect off.
      is_bypassed = 1;

    // If this hasn't been set (probably 0xff), we'll always assume it's ON_BOOT_REMEMBER
    // Any other value (including a blank 0xFF byte) is treated as "remember"
    } else {
      // Set the boot variable to "remember"
      on_boot_setting = ON_BOOT_REMEMBER;
      // Restore the last saved state (this also points eeprom_addr at the current byte).
      is_bypassed = eeprom_read_is_bypassed();
    }  

    // If switch is held when powered on, toggle the on_boot setting
    // CLAUDE: Footswitch held during power-up: step to the next power-on mode.

    *** REDO THE SETUP MODE START
    if (sw_last_loop == 0) {
      // CLAUDE: REMEMBER...
      if (on_boot_setting == ON_BOOT_REMEMBER) {
        // CLAUDE: ...becomes ENGAGE. This boot keeps the remembered state; the new mode applies from the next power-up.
        on_boot_setting = ON_BOOT_ENGAGE;
      // CLAUDE: ENGAGE...
      } else if (on_boot_setting == ON_BOOT_ENGAGE) {
        // CLAUDE: ...becomes BYPASS. This boot still starts engaged; the new mode applies from the next power-up.
        on_boot_setting = ON_BOOT_BYPASS;
      // CLAUDE: BYPASS...
      } else {
        // CLAUDE: ...wraps around to REMEMBER.
        on_boot_setting = ON_BOOT_REMEMBER;
        // More important than getting the is_bypassed setting is
        // setting the eeprom_addr, which this call does.
        // CLAUDE: Scan the EEPROM ring so eeprom_addr points at the current flagged byte before any writes.
        is_bypassed = eeprom_read_is_bypassed();
      }  // CLAUDE: End of the mode-stepping branches.
      // CLAUDE: Use the mode code as the blink count (the blink loop below actually blinks one fewer time).
      blink_n_times = on_boot_setting;
      *** REDO THE SETUP MODE END
      
      // CLAUDE: Save the new power-on mode to address 0 (written only if the value actually changed).
      eeprom_update_byte((uint8_t *)ADDR_BOOT_SETTING, on_boot_setting);
      
    }  // CLAUDE: End of the power-up mode-change block.

    // CLAUDE: Only in REMEMBER mode...
    if (on_boot_setting == ON_BOOT_REMEMBER) {
      // CLAUDE: ...enable saving each state change to EEPROM.
      use_eeprom = 1;
    }  // CLAUDE: End of the REMEMBER check.
  }  // CLAUDE: End of the EEPROM block.

  // CLAUDE: Pulse the relay to match is_bypassed and set the LED, before any blinking.
  write_bypass(); // get the relay setup before blinking the LED for EEPROM
  // CLAUDE: Short 1 ms pause after the relay pulse.
  _delay_ms(1);

  // CLAUDE: Only blink if the power-on mode was just changed.
  if (blink_n_times > 0) {
    // Blink LED. Ordinarily this should be done asynchronously,
    // but on initial boot it probably doesn't matter
    // CLAUDE: Counter starts at 1, so this blinks blink_n_times - 1 times: ENGAGE = 2, BYPASS = 3, REMEMBER = 4 blinks.
    for (int i = 1; i < blink_n_times; i++) {
      // CLAUDE: LED on.
      PORTB |= (1 << PIN_LED); //high
      // CLAUDE: Keep it on for 200 ms.
      _delay_ms(200);
      // CLAUDE: LED off.
      PORTB &= ~(1 << PIN_LED); //low
      // CLAUDE: Keep it off for 200 ms.
      _delay_ms(200);
    }  // CLAUDE: End of the blink loop.
    // CLAUDE: 1 s pause so the blinks are clearly separated from the normal LED state.
    _delay_ms(1000);
    // CLAUDE: Put the LED back to showing the actual engaged/bypassed state.
    set_led(); // set the LED again
  }  // CLAUDE: End of the blink block.
}  // CLAUDE: End of setup().

// CLAUDE: Arduino runs loop() over and over after setup().
void loop() {
  // CLAUDE: Raw footswitch reading for this pass (0 = pressed).
  uint8_t sw_this_loop = read_switch();

  // CLAUDE: Current time in ms since power-up.
  unsigned long now = millis();

// CLAUDE: The next block is compiled only for the traditional debounce.
#ifndef SIMPLIFIED_DEBOUNCE
  // If still unstable, reset the clock
  // CLAUDE: Raw reading differs from the previous pass (switch still bouncing)...
  if (sw_this_loop != sw_last_loop) {
    // CLAUDE: ...so restart the stability timer.
    sw_stable_since = now;
  }  // CLAUDE: End of the bounce check.
// CLAUDE: End of the traditional-debounce-only block.
#endif

  // has sw state has changed?
  // Is switch state stable?
  // CLAUDE: Accept a switch change only if the raw reading differs from the debounced state...
  if ((sw_this_loop != sw_state)
      // CLAUDE: ...and more than DEBOUNCE_DELAY ms have passed (stable for 14 ms, or the 250 ms lockout since the last accepted edge).
      && (now - sw_stable_since) > DEBOUNCE_DELAY)
  {  // CLAUDE: Start of the block that handles an accepted switch change.
// CLAUDE: The next line is compiled only for the simplified debounce.
#ifdef SIMPLIFIED_DEBOUNCE
    // CLAUDE: Start the 250 ms lockout from this accepted edge.
    sw_stable_since = now;
// CLAUDE: End of the simplified-debounce-only line.
#endif
    // CLAUDE: Record the new debounced switch state.
    sw_state = sw_this_loop;

    // CLAUDE: The change was a press (switch pulled low)...
    if (sw_state == 0) {// sw pressed
      // CLAUDE: ...note when it happened, for the hold-time check on release...
      sw_pressed_at = now;
      // CLAUDE: ...and flip engaged/bypassed right away, on the press.
      toggle_bypass_state();

    // Switch has just been released. Let's decide if it was a long press for temporary engagement/disengagement,
    // or a quick press to toggle its state.
    // CLAUDE: Otherwise the change was a release.
    } else {
// CLAUDE: The next block is compiled only if momentary (hold) operation is enabled.
#ifndef DISABLE_TEMPORARY_SWITCH
      // CLAUDE: Held longer than 500 ms? (After holding the switch at power-up, sw_pressed_at is still 0, so that release also counts as a hold.)
      if ((now - sw_pressed_at) > TEMPORARY_SWITCH_TIME) {
        // CLAUDE: Flip the state back, so a long hold acts like a momentary switch.
        toggle_bypass_state();
      }  // CLAUDE: End of the long-hold check.
// CLAUDE: End of the momentary-operation block.
#endif
    }  // CLAUDE: End of the press/release branches.
  }  // CLAUDE: End of the accepted-switch-change block.

  // CLAUDE: Save this pass's raw reading for comparison on the next pass.
  sw_last_loop = sw_this_loop;
}  // CLAUDE: End of loop().

// returns is_bypassed
// Finds the flagged byte in EEPROM, points eeprom_addr at it, and returns the saved state (0 or 1).
uint8_t eeprom_read_is_bypassed() {
  // This may not find the flag, so these two defaults are important.
  // Default location if no flagged byte is found.
  eeprom_addr = ADDR_BYPASS_SETTING;
  // Default return value (engaged) if no flagged byte is found.
  uint8_t rv = 0;

  // Walk every EEPROM chunk starting at address 0 (so the boot-setting byte is checked too).
  for (uint16_t i = 0; i < (EEPROM_SIZE / EEPROM_CHUNK_SIZE); i++) {
    // Turn the chunk index into a byte address.
    uint16_t addr = i * EEPROM_CHUNK_SIZE;

    // Read the byte at that address.
    uint8_t b = eeprom_read_byte((const uint8_t *)addr);
    // Bit 7 set means this is the current-state byte. A blank 0xFF byte also matches, so a fresh chip boots bypassed.
    if (b & EEPROM_ADDR_FLAG_MASK) {
      // Remember where it is so the next save continues from here.
      eeprom_addr = addr;

      // Keep only bit 0, which holds the saved bypass state.
      rv = b & 0x1; // remove the EEPROM_ADDR_FLAG_MASK bit
      // Stop at the first flagged byte.
      break;
    }  
  }  
  // Return the saved state (0 if nothing was found).
  return rv;
}  

// writes is_bypassed
// CLAUDE: Saves the state to the next EEPROM byte (wear leveling): unflag the old byte, write the new one.
void eeprom_write_is_bypassed(uint8_t setting) {
  // don't unnecessarily wear out the eeprom.
  // CLAUDE: Not in REMEMBER mode...
  if (use_eeprom == 0) {
    // CLAUDE: ...so skip saving.
    return;
  }  // CLAUDE: End of the use_eeprom check.

  // CLAUDE: Zero the byte holding the old state, which removes its flag (written only if not already 0).
  eeprom_update_byte((uint8_t *)eeprom_addr, 0x0); // Clear flag on previous location.
                                        // This will also clean up any default data in
                                        // the eeprom that coincidentally had the flag bit set.

  // CLAUDE: If there is room before the end of the EEPROM...
  if (eeprom_addr < (EEPROM_SIZE - EEPROM_CHUNK_SIZE)) {
    // CLAUDE: ...move to the next byte.
    eeprom_addr += EEPROM_CHUNK_SIZE;
  // CLAUDE: Otherwise (already at the last byte)...
  } else {
    // CLAUDE: ...wrap back to address 1 (address 0 is reserved for the boot setting).
    eeprom_addr = ADDR_BYPASS_SETTING;
  }  // CLAUDE: End of the address advance.
  // CLAUDE: Write the state with bit 7 set, marking this byte as the current one.
  eeprom_update_byte((uint8_t *)eeprom_addr, EEPROM_ADDR_FLAG_MASK | setting);
}  // CLAUDE: End of eeprom_write_is_bypassed().

// This may block. See comments
// CLAUDE: Flips engaged <-> bypassed (with optional muting) and saves the new state.
void toggle_bypass_state() {
  // CLAUDE: Only if muting is enabled (it isn't while USE_MUTE is 0)...
  if (use_mute) {
    // Is there any reason to do this asynchronously?
    // Until it seems there is, let's keep it MUCH simpler.
    // CLAUDE: ...drive the mute pin high to mute the audio...
    PORTB |= (1 << PIN_MUTE); // HIGH - engage mute
    // wait a while for mute to fully engage before toggling relay
    // CLAUDE: ...and wait 20 ms for the mute to take effect.
    _delay_ms(MUTED_RELAY_DELAY);
  }  // CLAUDE: End of the mute-on block.

  // CLAUDE: Invert the state (0 becomes 1, 1 becomes 0).
  is_bypassed = ! is_bypassed;
  // CLAUDE: Pulse the relay to the new state and update the LED (blocks for RELAY_SWITCH_TIME = 40 ms).
  write_bypass();

  // CLAUDE: Only if muting is enabled...
  if (use_mute) {
    // wait for the rest of the MUTE_LENGTH period
    // CLAUDE: ...wait another 15 ms. Total mute is ~75 ms (20 + 40 relay pulse + 15), not 35, because write_bypass() also blocks.
    _delay_ms(MUTE_LENGTH - MUTED_RELAY_DELAY);
    // Unmute by setting PIN_MUTE to low
    // CLAUDE: Release the mute.
    PORTB &= ~(1 << PIN_MUTE);

    // since we've been blocking, the switch may have been released.
    // CLAUDE: Resync the debounced switch state, since it may have changed while we were blocked.
    sw_state = read_switch();
  }  // CLAUDE: End of the mute-off block.

  // CLAUDE: Save the new state to EEPROM (does nothing unless in REMEMBER mode).
  eeprom_write_is_bypassed(is_bypassed);
}  // CLAUDE: End of toggle_bypass_state().

// CLAUDE: Sets the LED and pulses the latching relay to match is_bypassed.
void write_bypass() {
  // CLAUDE: Update the LED first.
  set_led();
  // CLAUDE: Make sure the bypass drive pin starts low.
  PORTB &= ~(1 << PIN_BYPASS); // low
  // CLAUDE: Make sure the engage drive pin starts low.
  PORTB &= ~(1 << PIN_ENGAGE); // low
  // There's a non-zero chance the blocking we do here could
  // interfere with the switch debouncing.
  // But with the simplified debounce it's a non-issue.
  // CLAUDE: Bypass wanted...
  if (is_bypassed) {
    // CLAUDE: ...drive the bypass pin high to flip the relay to bypass...
    PORTB |= (1 << PIN_BYPASS); // high
    // CLAUDE: ...hold it for 40 ms so the relay has time to switch...
    _delay_ms(RELAY_SWITCH_TIME);
    // CLAUDE: ...then drop it. A latching relay stays put with no current.
    PORTB &= ~(1 << PIN_BYPASS); // low
  // CLAUDE: Engaged wanted...
  } else {
    // CLAUDE: ...drive the engage pin high to flip the relay to engaged...
    PORTB |= (1 << PIN_ENGAGE); // high
    // CLAUDE: ...hold it for 40 ms...
    _delay_ms(RELAY_SWITCH_TIME);
    // CLAUDE: ...then drop it.
    PORTB &= ~(1 << PIN_ENGAGE); // low
  }  // CLAUDE: End of the relay-pulse branches.
}  // CLAUDE: End of write_bypass().

// CLAUDE: Shows the state on the LED: off when bypassed, on when engaged.
void set_led() {
  // CLAUDE: Bypassed...
  if (is_bypassed) {
    // CLAUDE: ...LED pin low (LED off).
    PORTB &= ~(1 << PIN_LED); // low
  // CLAUDE: Engaged...
  } else {
    // CLAUDE: ...LED pin high (LED on).
    PORTB |= (1 << PIN_LED); // high
  }  // CLAUDE: End of the LED branches.
}  // CLAUDE: End of set_led().

// Returns the raw footswitch reading: 0 = pressed, non-zero = released.
uint8_t read_switch() {
  // Read the port B input pins and keep only PB2: 0 if pulled low (pressed), 4 if high (released).
  uint8_t rv = PINB & (1 << PIN_FOOTSWITCH);
// The next line is compiled only for a normally-closed footswitch.
#ifdef NORMALLY_CLOSED
  // CLAUDE: Invert the reading so 0 still means "pressed".
  rv = ! rv;
// CLAUDE: End of the normally-closed block.
#endif
  // CLAUDE: Return the reading.
  return rv;
}  // CLAUDE: End of read_switch().
