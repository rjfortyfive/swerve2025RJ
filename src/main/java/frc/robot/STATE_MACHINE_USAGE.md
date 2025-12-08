# State Machine Usage Guide

This project uses a state-based architecture on top of the command-based framework.

## Overview

The `StateManager` subsystem tracks the robot's current operational state. Commands can check the state before executing, and state transitions can be managed automatically.

## Robot States

- **IDLE**: Robot is idle, ready for commands
- **INTAKING**: Intaking coral from station
- **SCORING_L1/L2/L3/L4**: Scoring at specific levels
- **CLIMBING**: Climbing/hanging
- **MANUAL**: Manual control mode (bypasses state checks)

## Basic Usage

### 1. State-Aware Commands

Use the state-aware command wrappers that automatically manage state transitions:

```java
// In RobotContainer.configureBindings()
buttonPanel.button(Constants.buttonPanel.coral.IN)
    .onTrue(StateAwareCoralIntake.create(
        m_stateManager, m_elevator, m_effector, m_intake
    ));

buttonPanel.button(Constants.buttonPanel.coral.OUT)
    .onTrue(StateAwareScore.create(
        m_stateManager, m_elevator, m_effector, 4  // Score at L4
    ));
```

### 2. Manual State Transitions

Transition states manually:

```java
// Transition to a specific state
new SetStateCommand(m_stateManager, RobotState.INTAKING)

// Force a state transition (bypasses validation)
new SetStateCommand(m_stateManager, RobotState.IDLE, true)
```

### 3. State-Conditional Commands

Only run commands if in a specific state:

```java
// Only run if in INTAKING state
StateConditionalCommand.create(
    m_stateManager,
    RobotState.INTAKING,
    new SomeCommand(),
    Commands.print("Not in intaking state!")
)
```

### 4. Checking State in Commands

Commands can check state before executing:

```java
public class MyCommand extends Command {
    private final StateManager m_stateManager;
    
    @Override
    public void initialize() {
        if (m_stateManager.getState() != RobotState.IDLE) {
            // Handle invalid state
            cancel();
        }
    }
}
```

## State Transition Rules

The state machine enforces valid transitions:
- Can always transition to/from IDLE or MANUAL
- Can transition between scoring levels
- Can transition from INTAKING to IDLE or scoring states
- Can transition from scoring states to IDLE or other scoring states
- Can transition from CLIMBING to IDLE

## SmartDashboard

The current and previous states are published to SmartDashboard:
- `StateManager/CurrentState`
- `StateManager/PreviousState`

## Example: Converting Existing Commands

**Before (command-based only):**
```java
buttonPanel.button(Constants.buttonPanel.coral.IN)
    .onTrue(new CoralIntake(m_elevator, m_effector, m_intake));
```

**After (state-aware):**
```java
buttonPanel.button(Constants.buttonPanel.coral.IN)
    .onTrue(StateAwareCoralIntake.create(
        m_stateManager, m_elevator, m_effector, m_intake
    ));
```

This automatically:
1. Transitions to INTAKING state
2. Runs the intake command
3. Returns to IDLE when done

