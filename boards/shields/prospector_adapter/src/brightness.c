#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/led.h>
#include <zephyr/sys/printk.h>

// Code Added for Manual Brightness Change
#include <zmk/event_manager.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/events/layer_state_changed.h>
#include <math.h>
#include <stdlib.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(als, 4);

static const struct device *pwm_leds_dev = DEVICE_DT_GET_ONE(pwm_leds);
#define DISP_BL DT_NODE_CHILD_IDX(DT_NODELABEL(disp_bl))

// Manual Brightness Control
#define BRIGHTNESS_STEP 1
#define BRIGHTNESS_DELAY_MS 2
#define BRIGHTNESS_FADE_DURATION_MS 500
#define SCREEN_IDLE_TIMEOUT_MS (CONFIG_DONGLE_SCREEN_IDLE_TIMEOUT_S * 1000)
#define BRIGHTNESS_CHANGE_THRESHOLD 5

static int64_t last_activity = 0;
static uint8_t max_brightness = CONFIG_DONGLE_SCREEN_MAX_BRIGHTNESS;
static uint8_t min_brightness = CONFIG_DONGLE_SCREEN_MIN_BRIGHTNESS;
static int8_t current_brightness = CONFIG_DONGLE_SCREEN_DEFAULT_BRIGHTNESS;

static int8_t brightness_modifier = CONFIG_DONGLE_SCREEN_BRIGHTNESS_MODIFIER;

static bool off_through_modifier = false; // Used to track if the screen was turned off through the brightness modifier

/**
 * @brief Structure to hold brightness calculation results
 */
struct brightness_result
{
    uint8_t adjusted_brightness;  // The adjusted base brightness value
    int8_t adjusted_modifier;     // The adjusted modifier value
    uint8_t effective_brightness; // Final brightness (adjusted_brightness + adjusted_modifier)
    bool was_clamped;             // Whether any clamping occurred
    bool hit_min_limit;           // Whether minimum limit was reached
    bool hit_max_limit;           // Whether maximum limit was reached
};

static uint8_t clamp_brightness(int8_t value)
{
    if (value > max_brightness) {return max_brightness;}
    if (value < min_brightness) {return min_brightness;}
    return value;
}

static void apply_brightness(uint8_t value)
{
    led_set_brightness(pwm_leds_dev, DISP_BL, value);
}

static int8_t calculate_safe_modifier_change(uint8_t base_brightness, int8_t current_modifier, int8_t desired_change)
{
    int16_t current_effective = base_brightness + current_modifier;
    int16_t desired_effective = current_effective + desired_change;

    // Determine the appropriate boundary based on direction of change
    int16_t boundary = (desired_change > 0) ? max_brightness : min_brightness;

    // Check if the desired change is within bounds
    if ((desired_change > 0 && desired_effective <= boundary) ||
        (desired_change < 0 && desired_effective >= boundary))
    {
        return desired_change; // Full change is safe
    }

    // Calculate maximum safe change toward the boundary
    int16_t safe_change = boundary - current_effective;

    // Ensure we don't return a change in the wrong direction or zero when some change is possible
    if ((desired_change > 0 && safe_change > 0) || (desired_change < 0 && safe_change < 0))
    {
        return (int8_t)safe_change;
    }

    return 0; // No safe change possible
}

static struct brightness_result calculate_brightness_with_bounds(uint8_t base_brightness, int8_t modifier, bool enforce_ambient_constraints)
{
    struct brightness_result result = {0};

    // Start with clamped base brightness
    result.adjusted_brightness = clamp_brightness(base_brightness);
    result.adjusted_modifier = modifier;

    // Recalculate effective brightness after any ambient adjustments
    result.effective_brightness = clamp_brightness(result.adjusted_brightness + result.adjusted_modifier);

    // Final boundary check
    if (result.effective_brightness <= min_brightness)
    {
        result.hit_min_limit = true;
    }
    if (result.effective_brightness >= max_brightness)
    {
        result.hit_max_limit = true;
    }

    return result;
}

static bool should_screen_turn_off(uint8_t base_brightness, int8_t modifier)
{
    return (base_brightness + modifier) < min_brightness;
}

static bool should_screen_turn_on(uint8_t base_brightness, int8_t modifier)
{
    return (base_brightness + modifier) > min_brightness;
}

// Threaded fade logic
// Contains starting and target brightness levels to be animated
struct fade_request_t
{
    uint8_t from; // Starting brightness level
    uint8_t to;   // Target brightness level
};

#define FADE_QUEUE_SIZE 4

// Message queue used to send fade requests to the fade handler thread.
// It holds up to 4 fade_request_t elements and ensures brightness updates are handled sequentially.
K_MSGQ_DEFINE(fade_msgq, sizeof(struct fade_request_t), FADE_QUEUE_SIZE, 4);

// Cubic ease-in-out function to smooth the interpolation curve.
// Provides a natural "S-curve" animation effect: starts slow, accelerates, then slows again.
// Helps avoid abrupt changes in perceived brightness.
static float ease_in_out(float t)
{
    if (t < 0.5f)
        return 4.0f * t * t * t;
    float f = -2.0f * t + 2.0f;
    return 1.0f - (f * f * f) / 2.0f;
}

// Dedicated thread responsible for handling all fade animations.
// Receives fade requests from the queue and applies brightness changes over time using easing.
void fade_thread(void)
{
    struct fade_request_t req;

    while (1)
    {
        // Wait indefinitely for the next fade request to arrive in the queue
        if (k_msgq_get(&fade_msgq, &req, K_FOREVER) == 0)
        {

            // Skip animation entirely if brightness difference is too small
            if (req.from == req.to || abs(req.to - req.from) <= 1)
            {
                apply_brightness(req.to);
                continue;
            }

            // Calculate brightness difference and use it to determine number of steps
            int diff = abs(req.to - req.from);
            int steps = CLAMP(diff * 2, 6, 32); // More steps for smoother fades over large differences

            // Set total animation time: scale with difference but clamp between 500ms and 1000ms
            int total_duration_ms = CLAMP(diff * 20, 500, 1000); // 20ms per level as baseline
            int delay_us = (total_duration_ms * 1000) / steps;   // Delay between steps in microseconds

            uint8_t last_applied = 255; // Used to prevent redundant LED updates to save performance

            // Interpolate brightness across 'steps' frames using easing
            for (int i = 0; i <= steps; i++)
            {
                float t = (float)i / steps;                                  // Normalized time in [0, 1]
                float eased = ease_in_out(t);                                // Eased time for smoother progression
                float interpolated = req.from + (req.to - req.from) * eased; // Interpolated value
                uint8_t brightness = (uint8_t)(interpolated + 0.5f);         // Rounded to nearest integer

                // Only send update if brightness actually changed
                if (brightness != last_applied)
                {
                    apply_brightness(brightness);
                    last_applied = brightness;
                }

                k_usleep(delay_us); // Sleep before next step to pace the fade
            }

            // safeguard to ensure the target value is set at the end
            if (last_applied != req.to)
            {
                apply_brightness(req.to);
            }
        }
    }
}

// Launch the fade thread with 768 bytes of stack, medium priority (6)
// 512 was too small for logging, math (float, int), small loop, few stack-local variables
// 768 is just a guess, optimization is possible, probably
K_THREAD_DEFINE(fade_tid, 768, fade_thread, NULL, NULL, NULL, 6, 0, 0);

// Function to submit a brightness fade request
// Ensures that only the most recent fade request is applied by purging the queue first for changes in between animations
static void fade_to_brightness(uint8_t from, uint8_t to)
{
    struct fade_request_t req = {.from = from, .to = to};
    k_msgq_purge(&fade_msgq);                // Clear any pending fades to avoid outdated transitions
    k_msgq_put(&fade_msgq, &req, K_NO_WAIT); // Submit the new fade request without blocking
}

void set_screen_brightness(uint8_t value, bool ambient)
{
    struct brightness_result result = calculate_brightness_with_bounds(value, brightness_modifier, ambient);

    uint8_t current_effective = clamp_brightness(current_brightness + brightness_modifier);

    fade_to_brightness(current_effective, result.effective_brightness);
    current_brightness = result.adjusted_brightness;
}

#if CONFIG_DONGLE_SCREEN_IDLE_TIMEOUT_S > 0 || CONFIG_DONGLE_SCREEN_BRIGHTNESS_KEYBOARD_CONTROL
// --- Brightness logic ---
static bool screen_on = true;
// --- Screen on/off ---

static void screen_set_on(bool on)
{
    if (on && !screen_on)
    {
        // Use unified helper to check if we need brightness adjustment
        if (should_screen_turn_off(current_brightness, brightness_modifier))
        {
            struct brightness_result result = calculate_brightness_with_bounds(current_brightness, brightness_modifier, false);
            current_brightness = result.adjusted_brightness;
        }

        fade_to_brightness(0, clamp_brightness(current_brightness + brightness_modifier));
        screen_on = true;
        off_through_modifier = false; // Reset the flag, because the screen is turned on again
    }
    else if (!on && screen_on)
    {
        fade_to_brightness(clamp_brightness(current_brightness + brightness_modifier), 0);
        screen_on = false;
    }
    else
    {
        LOG_DBG("Screen state is already %s, no action taken.", on ? "on" : "off");
    }
}

#endif

// --- Idle thread ---

#if CONFIG_DONGLE_SCREEN_IDLE_TIMEOUT_S > 0

void screen_idle_thread(void)
{
    while (1)
    {
        // Thread should run even if the screen is off, but only if the screen is off through the modifier
        if (screen_on || (!screen_on && off_through_modifier))
        {
            int64_t now = k_uptime_get();
            int64_t elapsed = now - last_activity;
            int64_t remaining = SCREEN_IDLE_TIMEOUT_MS - elapsed;

            if (remaining <= 0)
            {
                screen_set_on(false);
                off_through_modifier = false; // Reset the flag, because the screen is turned off
                // After turning off, sleep until next activity (key event will wake screen)
                k_sleep(K_FOREVER);
            }
            else
            {
                // Sleep exactly as long as needed until timeout or next key event
                k_sleep(K_MSEC(remaining));
            }
        }
        else
        {
            // If Screen is off, sleep forever (will be interrupted by key event)
            k_sleep(K_FOREVER);
        }
    }
}

K_THREAD_DEFINE(screen_idle_tid, 512, screen_idle_thread, NULL, NULL, NULL, 7, 0, 0);

void brightness_wake_screen_on_reconnect(void)
{
    if (!screen_on)
    {
        screen_set_on(true);

        // Reset idle timer
        last_activity = k_uptime_get();

        k_wakeup(screen_idle_tid);
    }
    else
    {
        LOG_DBG("Peripheral reconnected but screen already on");
    }
}

#endif

// --- Brightness control via keyboard ---

#if CONFIG_DONGLE_SCREEN_BRIGHTNESS_KEYBOARD_CONTROL

static void increase_brightness(void)
{
    int8_t safe_increase = calculate_safe_modifier_change(current_brightness, brightness_modifier, CONFIG_DONGLE_SCREEN_BRIGHTNESS_STEP);

    if (safe_increase > 0)
    {
        brightness_modifier += safe_increase;
        set_screen_brightness(current_brightness, false);

        // Check if we should turn screen on
        if (should_screen_turn_on(current_brightness, brightness_modifier) && off_through_modifier)
        {
            screen_set_on(true);
        }
    }
    else
    {
        LOG_DBG("Brightness modifier cannot be increased further (at maximum)");
    }
}

static void decrease_brightness(void)
{
    int8_t safe_decrease = calculate_safe_modifier_change(current_brightness, brightness_modifier, -CONFIG_DONGLE_SCREEN_BRIGHTNESS_STEP);

    if (safe_decrease < 0)
    {                                         // safe_decrease will be negative for decreases
        brightness_modifier += safe_decrease; // Adding a negative value decreases
        set_screen_brightness(current_brightness, false);

        // Check if we should turn screen off
        if (should_screen_turn_off(current_brightness, brightness_modifier))
        {
            off_through_modifier = true;
            screen_set_on(false);
        }
    }
    else
    {
        LOG_DBG("Brightness modifier cannot be decreased further (at minimum)");
    }
}

#endif // CONFIG_DONGLE_SCREEN_BRIGHTNESS_KEYBOARD_CONTROL

#if CONFIG_DONGLE_SCREEN_IDLE_TIMEOUT_S > 0 || CONFIG_DONGLE_SCREEN_BRIGHTNESS_KEYBOARD_CONTROL

// --- Key event listener ---

static int key_listener(const zmk_event_t *eh)
{
    const struct zmk_keycode_state_changed *ev = as_zmk_keycode_state_changed(eh);
    if (ev && ev->state)
    { // Only on key down
        LOG_DBG("Key pressed: keycode=%d", ev->keycode);

#if CONFIG_DONGLE_SCREEN_BRIGHTNESS_KEYBOARD_CONTROL
        if (ev->keycode == CONFIG_DONGLE_SCREEN_BRIGHTNESS_UP_KEYCODE)
        {
            increase_brightness();
            return 0;
        }
        else if (ev->keycode == CONFIG_DONGLE_SCREEN_BRIGHTNESS_DOWN_KEYCODE)
        {
            decrease_brightness();
            return 0;
        }
        else if (ev->keycode == CONFIG_DONGLE_SCREEN_TOGGLE_KEYCODE)
        {
            // Toggle screen on/off
            if (screen_on)
            {
                off_through_modifier = true; // Track that the screen was turned off through the toggle key
                screen_set_on(false);
            }
            else
            {
                screen_set_on(true);
            }
            return 0;
        }

#endif
    }

#if CONFIG_DONGLE_SCREEN_IDLE_TIMEOUT_S > 0
    last_activity = k_uptime_get();
    if (!screen_on && !off_through_modifier)
    {
        screen_set_on(true);
        k_wakeup(screen_idle_tid);
    }
#else
    // Without idle thread: just turn on screen
    if (!screen_on)
    {
        screen_set_on(true);
    }
#endif
    return 0;
}

ZMK_LISTENER(screen_idle, key_listener);
ZMK_SUBSCRIPTION(screen_idle, zmk_keycode_state_changed);
ZMK_SUBSCRIPTION(screen_idle, zmk_layer_state_changed);

#endif

// Ambient Light Control

#ifdef CONFIG_PROSPECTOR_USE_AMBIENT_LIGHT_SENSOR

static uint8_t current_brightness = 100;

#define SENSOR_MIN      0       // Minimum sensor reading
#define SENSOR_MAX      100     // Maximum sensor reading
#define PWM_MIN         1       // Minimum PWM duty cycle (%) - keep display visible
#define PWM_MAX         100     // Maximum PWM duty cycle (%)

#define FADE_STEP                        1
#define FADE_SLEEP_BRIGHTEN_MS           3
#define FADE_SLEEP_DARKEN_MS             10
#define FADE_THRESHOLD                   10

#define NORMAL_SAMPLE_SLEEP_MS           100

#define BURST_SAMPLE_SLEEP_MS            30
#define BURST_SAMPLE_TIMEOUT             10
#define BURST_SAMPLE_CONSECUTIVE         3

uint8_t map_light_to_pwm(int32_t sensor_reading) {
    // Handle invalid/error readings
    if (sensor_reading < SENSOR_MIN) {
        return PWM_MIN;  // Default to minimum brightness on error
    }

    // Clamp to maximum
    if (sensor_reading > SENSOR_MAX) {
        sensor_reading = SENSOR_MAX;
    }

    // Linear mapping
    uint8_t pwm_value = (uint8_t)(
        PWM_MIN + ((PWM_MAX - PWM_MIN) *
        (sensor_reading - SENSOR_MIN)) / (SENSOR_MAX - SENSOR_MIN)
    );

    return pwm_value;
}

uint8_t bl_fade(uint8_t source, uint8_t target) {
    bool increasing = target > source;

    while ((increasing && current_brightness < target) ||
           (!increasing && current_brightness > target)) {

        if (led_set_brightness(pwm_leds_dev, DISP_BL, current_brightness)) {
            LOG_ERR("Failed to set brightness");
        }

        current_brightness += increasing ? FADE_STEP : -FADE_STEP;

        // Ensure we don't overshoot bounds
        if (current_brightness > 100) {
            current_brightness = 100;
        } else if (current_brightness < 0) {
            current_brightness = 0;
        }

        k_msleep(increasing ? FADE_SLEEP_BRIGHTEN_MS : FADE_SLEEP_DARKEN_MS);
    }

    return 0;
}

extern void als_thread(void *d0, void *d1, void *d2) {
    ARG_UNUSED(d0);
    ARG_UNUSED(d1);
    ARG_UNUSED(d2);

    const struct device *dev;
    struct sensor_value intensity;
    uint8_t mapped_brightness;

    dev = DEVICE_DT_GET_ONE(avago_apds9960);
    if (!device_is_ready(dev)) {
        printk("sensor: device not ready.\n");
    }

    // led_set_brightness(pwm_leds_dev, DISP_BL, 100);

    while (1) {

        k_msleep(NORMAL_SAMPLE_SLEEP_MS);


        if (sensor_sample_fetch(dev)) {
            LOG_ERR("sensor_sample fetch failed\n");
        }

        if (sensor_channel_get(dev, SENSOR_CHAN_LIGHT, &intensity)) {
            LOG_ERR("Cannot read ALS data.\n");
        }

        // LOG_INF("ambient light intensity %d", intensity.val1);

        mapped_brightness = map_light_to_pwm(intensity.val1);
        // LOG_INF("NORMAL: mapped PWM duty cycle %d\n", mapped_brightness);

        if (abs(mapped_brightness - current_brightness) > FADE_THRESHOLD) {
            uint8_t integrator = 0;

            for (int i = 0; i < BURST_SAMPLE_TIMEOUT; i++) {
                k_msleep(BURST_SAMPLE_SLEEP_MS);

                if (sensor_sample_fetch(dev)) {
                    LOG_ERR("sensor_sample fetch failed\n");
                }
                if (sensor_channel_get(dev, SENSOR_CHAN_LIGHT, &intensity)) {
                    LOG_ERR("Cannot read ALS data.\n");
                }

                mapped_brightness = map_light_to_pwm(intensity.val1);
                // LOG_INF("BURST: mapped PWM duty cycle %d\n", mapped_brightness);

                if (abs(mapped_brightness - current_brightness) > FADE_THRESHOLD) {
                    integrator++;
                    // printk("integrator at: %d", integrator);
                    if (integrator >= BURST_SAMPLE_CONSECUTIVE) {
                        bl_fade(current_brightness, mapped_brightness);
                        current_brightness = mapped_brightness;
                        // LOG_INF("SETTING NEW BRIGHTNESS: %d", mapped_brightness);
                        break;
                    }
                }
            }
        }
        // led_set_brightness(pwm_leds_dev, DISP_BL, map_light_to_pwm(intensity.val1));
    }
}

K_THREAD_DEFINE(als_tid, 1024, als_thread, NULL, NULL, NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0,
                0);

#endif

// --- Initialization ---

static int init_fixed_brightness(void)
{
    set_screen_brightness(current_brightness, false);
    last_activity = k_uptime_get();
#if CONFIG_DONGLE_SCREEN_IDLE_TIMEOUT_S > 0
    // Wake up the idle thread at boot
    k_wakeup(screen_idle_tid);
#else
    LOG_INF("Screen idle timeout disabled");
#endif
    return 0;
}

SYS_INIT(init_fixed_brightness, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

