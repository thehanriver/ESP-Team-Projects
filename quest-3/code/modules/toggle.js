export var led_status = 0;

export function toggleLED() {
    if (!led_status)
    {
      led_status=1;
    }
    else
    {
      led_status=0
    }
}

