# Electronics

Document the actual, verified wiring here. Do not infer connector pinouts from software constants.

## Required blocks

- Jetson Orin Nano + carrier and camera(s)
- Body MCU: project decision is STM32F401/F411 Blackpill (verify exact board before assigning pins)
- Motor/ESC and steering servo
- Battery, regulators, grounds, fusing, and power distribution
- Separate physical start and easily reachable emergency-stop controls

## Safety interface requirements

- E-stop must inhibit propulsion independently of Jetson software and remain latched until deliberate reset.
- MCU defaults to motor disabled at reset and on stale/invalid Jetson setpoints.
- Start is accepted only when e-stop is released and the system is ready; after an e-stop, require a fresh start action.
- No wired or wireless communications during an attempt.

Pin map, schematic, wire gauges, fuse values, voltage/current ratings, and measured tests are intentionally blank until the real components and wiring are confirmed.
