# BOOBUZZ robot-code

Robot software for Zağanos #25577's BIOBUZZ 2026–27 FTC season. The Android app
runs on the robot; the pure-Java core is also the bytecode used by the Java
simulator client.

## Modules

- `:core` lives at `TeamCode/core`. It is a Java 17 library containing the HAL
  contracts, controller, subsystem, and `cplx1` logic. It has no FTC or
  Android dependency; `gradle/sdk-guard.gradle` enforces that boundary.
- `:sim` is the Java 17 command-line simulator adapter (`SimMain`). It depends
  on `:core`, never on `TeamCode`, and speaks the line-delimited JSON protocol
  to the sibling `re-cock-nize` Python server.
- `:TeamCode` is the FTC Android app. Its SDK-facing HAL and OpModes depend on
  `:core` and `FtcRobotController`.

Robot configuration is compile-time Java in
`TeamCode/core/src/main/java/boobuzz/core/hal/RobotConstants.java`; the Python
simulator reads that same file.

## Build and tests

Use Java 21 for the Gradle wrapper (the modules target Java 17):

```bash
export JAVA_HOME=/usr/lib/jvm/java-21-openjdk
./gradlew :core:test :sim:test
./gradlew :TeamCode:assembleDebug
```

## Simulator end-to-end

Start the sibling Python server in one terminal:

```bash
cd ../re-cock-nize
.venv/bin/python -m sim.server \
  --mechanism ../robot-code/TeamCode/core/src/main/java/boobuzz/core/hal/RobotConstants.java \
  --physics pymunk --headless --port 5556
```

Then run the Java client from this repository in another terminal:

```bash
cd ../robot-code
export JAVA_HOME=/usr/lib/jvm/java-21-openjdk
./gradlew :sim:run --args="--port 5556 --path test-line --x 72 --y 72 --h 0 --dt 20 --steps 1000 --seed 1"
```

The server also supports `--physics pybullet|kinematic`, `--gui`, and the
headless mode documented in the journey repository.

## Documentation

Architecture, protocol, plans, and phase history live in the
[`boobuzz-docs`](https://github.com/tunapro1234/boobuzz-docs) repository.
