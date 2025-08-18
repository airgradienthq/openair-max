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
11. Quick blink 5x when trigger to start/stop system settings portal
12. Quick blink (forever) when system settings portal is currently active

### Every wakeup cycle

1. Led indicator is off unless there's error
2. Slow blink 2x if there's one or more sensor failed to initialize
3. Slow blink 3x when failed post to server

## System Settings Portal

If, on the first boot, the **boot button** is held for 5 seconds or longer, the MAX will launch the _System Settings Portal_. From there, you can configure settings such as:

- Switching the transmission mode between cellular and Wi-Fi  
- Setting the cellular APN  
- Entering Wi-Fi credentials  

To exit, either **save the new settings**, press the **Exit** button on the home page, or hold the **boot button** again for 5 seconds or longer.  

