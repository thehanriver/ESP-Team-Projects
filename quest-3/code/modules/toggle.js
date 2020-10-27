var led_status = 0;

function toggleLED() {
    if (!led_status)
    {
      led_status=1;
    }
    else
    {
      led_status=0
    }
}

export {led_status, toggleLED};
