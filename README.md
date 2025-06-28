# Firmware for the Open Air Max, uses airgradient-client and airgradient-ota

## Led indicator

### Powered On (1st boot)

1. Led indicator turned on right away
2. Quick blink 2x when starting initialize CE card and attempt to connect to network
3. Quick blink 3x when successfully connect to network
4. Quick blink 4x when successfully post 1st measurement to server
5. Slow blink (500ms interval) 2x if there's one or more sensor failed to initialize
6. Slow blink (500ms interval) 3x when failed post to server
7. Slow blink (1000ms interval) 5x when failed connect to network before retry

### Every wakeup cycle

1. Led indicator is off unless there's error
2. Slow blink (500ms interval) 2x if there's one or more sensor failed to initialize
3. Slow blink (500ms interval) 3x when failed post to server
