# Elevator Health Monitoring System

A system for monitoring elevator health and movement using an inertial measurement unit (IMU).

## Overview

This project implements a real-time elevator monitoring system that tracks:
- Elevator movement state (up, down, stationary)
- Acceleration data
- Altitude changes
- Speed calculations

## Hardware Components

The system uses the following sensors and components:
- AT32F425 microcontroller
- SPL06 barometric pressure sensor
- LIS2DH12 accelerometer
- TMF8001 time-of-flight sensor

## Implementation Details

The system uses a Kalman filter to process sensor data and determine the elevator's state. It calculates:
- Altitude based on barometric pressure
- Acceleration in three axes
- Average speed
- Movement state (UP, DOWN, STATIONARY, UNKNOWN)

### Key Features

- Real-time monitoring of elevator movement
- Accurate state detection using accelerometer data
- Altitude calculation using barometric pressure
- Data filtering for improved accuracy

## Code Example

The main detection algorithm uses acceleration thresholds to determine elevator state:

```c
int get_accelerate_status(int last_state, float baseValue, float data)
{
    float delta = 0;
    float average_data = 0;
    delta = data - baseValue;
    int state = last_state;
    accelerate_sign = get_accelerate_sign(baseValue, data);
    if (accelerate_sign != last_accelerate_sign)
    {
        if (accelerate_moving_state == ELEVATOR_MOVE_UNKNOWN)
        {
            if (accelerate_sign != STATE_SIGN_ZERO)
            {
                accelerate_moving_state = ELEVATOR_MOVE_MOVING;
                if (accelerate_sign == STATE_SIGN_POSITIVE)
                {
                    state = ELEVATOR_STATE_UP;
                }
                else if (accelerate_sign == STATE_SIGN_NEGATIVE)
                {
                    state = ELEVATOR_STATE_DOWN;
                }
            }
        }
        else if (accelerate_moving_state == ELEVATOR_MOVE_MOVING)
        {
            if (accelerate_sign != STATE_SIGN_ZERO)
            {
                accelerate_moving_state = ELEVATOR_MOVE_UNKNOWN;
                state = ELEVATOR_STATE_STATIONARY;
            }
        }
        else if (accelerate_sign == STATE_SIGN_ZERO)
        {
            if (accelerate_moving_state != ELEVATOR_MOVE_MOVING)
            {
                state = ELEVATOR_STATE_STATIONARY;
            }
        }
    }

    last_accelerate_sign = accelerate_sign;
    return state;
}
```

## Getting Started

To compile and run this project, you'll need:
1. AT32F425 development environment
2. Connected sensors (SPL06, LIS2DH12, TMF8001)
3. Appropriate power supply

See the [main.c](main.c) file for implementation details.