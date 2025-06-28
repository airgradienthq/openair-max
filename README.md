# Firmware for the Open Air Max, uses airgradient-client and airgradient-ota

## Led indicator

### Powered On (1st boot)

1. Led indicator turned on right away
2. Quick blink 2x when starting initialize CE card and attempt to connect to network
3. Quick blink 3x when successfully connect to network
4. Quick blink 4x when successfully post 1st measurement to server
5. Slow blink 2x if there's one or more sensor failed to initialize
6. Slow blink 3x when failed post to server
7. Slow blink 4x when failed post to server because of server issue
8. Slow blink 5x when ensure connection failed after unsuccessful post to server
9. Slow blink 6x when give up after 3 attempts of failed post measures because of network reasons
10. Slow blink 10x when failed connect to network before retry

### Every wakeup cycle

1. Led indicator is off unless there's error
2. Slow blink 2x if there's one or more sensor failed to initialize
3. Slow blink 3x when failed post to server
