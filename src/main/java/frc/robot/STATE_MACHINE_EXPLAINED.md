# State Machine Explained - Complete Walkthrough

## 🎯 What is a State Machine?

A state machine is a way to track what the robot is currently doing. Think of it like a traffic light:
- **Red** = IDLE (robot is stopped, ready)
- **Yellow** = INTAKING (robot is getting coral)
- **Green** = SCORING (robot is placing coral)

The state machine ensures the robot knows what it's doing and prevents conflicting actions.

---

## 🏗️ Architecture Overview

```
┌─────────────────────────────────────────────────┐
│           Command-Based Framework               │
│  (Your existing commands: CoralIntake, etc.)   │
└─────────────────────────────────────────────────┘
                    ↕
┌─────────────────────────────────────────────────┐
│            State Machine Layer                  │
│  (StateManager tracks current robot state)      │
└─────────────────────────────────────────────────┘
                    ↕
┌─────────────────────────────────────────────────┐
│              Physical Subsystems                 │
│  (Elevator, Effector, Intake, Drivetrain)       │
└─────────────────────────────────────────────────┘
```

The state machine sits **on top** of your command-based framework. It doesn't replace commands - it adds organization and safety.

---

## 📦 Component Breakdown

### 1. **StateManager Subsystem** - The Brain

**Location:** `subsystems/StateManager.java`

**What it does:**
- Stores the current robot state (IDLE, INTAKING, SCORING_L1, etc.)
- Validates state transitions (prevents invalid changes)
- Provides helper methods to check current state

**Key Fields:**
```java
private RobotState currentState = RobotState.IDLE;  // Where we are now
private RobotState previousState = RobotState.IDLE; // Where we were
```

**Key Methods:**
- `getState()` - Returns current state
- `setState(newState)` - Tries to change state (validates first)
- `forceState(newState)` - Forces state change (bypasses validation)
- `isIntaking()`, `isScoring()`, etc. - Convenience checkers

---

### 2. **RobotState Enum** - The States

**What are the states?**

```java
IDLE          // Robot is doing nothing, ready for commands
INTAKING      // Robot is intaking coral from station
SCORING_L1    // Robot is scoring at level 1
SCORING_L2    // Robot is scoring at level 2
SCORING_L3    // Robot is scoring at level 3
SCORING_L4    // Robot is scoring at level 4
CLIMBING      // Robot is climbing/hanging
MANUAL        // Manual mode - bypasses all state checks
```

**Why these states?**
- Each represents a major robot operation
- Helps prevent conflicts (e.g., can't intake while scoring)
- Makes debugging easier (you can see what the robot thinks it's doing)

---

### 3. **State Transition Rules** - The Safety System

**How transitions work:**

The `isValidTransition()` method enforces rules:

```
✅ Always Allowed:
   - Any state → IDLE
   - Any state → MANUAL
   - IDLE → Any state
   - MANUAL → Any state

✅ Specific Rules:
   - INTAKING → IDLE or SCORING states
   - SCORING states → IDLE or other SCORING states
   - CLIMBING → IDLE only
   - SCORING states can switch between each other

❌ Not Allowed:
   - INTAKING → CLIMBING (must go through IDLE first)
   - CLIMBING → SCORING (must go through IDLE first)
```

**Example Flow:**
```
IDLE → INTAKING → SCORING_L4 → IDLE  ✅ Valid
INTAKING → CLIMBING  ❌ Invalid (must go IDLE first)
```

---

### 4. **SetStateCommand** - The Transition Tool

**Location:** `commands/SetStateCommand.java`

**What it does:**
- A command that changes the robot's state
- Can be used in button bindings or command sequences
- Validates transitions (unless forced)

**How it works:**
```java
// Normal transition (validates)
new SetStateCommand(m_stateManager, RobotState.INTAKING)

// Forced transition (bypasses validation - use carefully!)
new SetStateCommand(m_stateManager, RobotState.IDLE, true)
```

**Lifecycle:**
1. `initialize()` - Changes the state immediately
2. `isFinished()` - Returns true immediately (instant command)
3. Done!

---

### 5. **StateAwareCoralIntake** - The Wrapper

**Location:** `commands/StateAwareCoralIntake.java`

**What it does:**
- Wraps your existing `CoralIntake` command
- Automatically manages state transitions
- Creates a command sequence

**How it works:**
```java
Commands.sequence(
    // Step 1: Transition to INTAKING state
    new SetStateCommand(stateManager, RobotState.INTAKING),
    
    // Step 2: Run the actual intake command
    new CoralIntake(elevator, effector, intake),
    
    // Step 3: Return to IDLE when done
    new SetStateCommand(stateManager, RobotState.IDLE)
)
```

**Visual Flow:**
```
Button Pressed
    ↓
[SetState: IDLE → INTAKING]
    ↓
[CoralIntake runs - elevator moves, intake spins, etc.]
    ↓
[CoralIntake finishes]
    ↓
[SetState: INTAKING → IDLE]
    ↓
Done!
```

---

## 🔄 Complete Example: Intake Button Pressed

Let's trace what happens when you press the intake button:

### **Before State Machine:**
```
Button Pressed → CoralIntake runs → Done
```

### **With State Machine (using StateAwareCoralIntake):**

```
1. Button Pressed
   └─> StateAwareCoralIntake.create() is called

2. Command Sequence Starts
   └─> SetStateCommand.initialize()
       └─> StateManager.setState(RobotState.INTAKING)
           └─> isValidTransition(IDLE, INTAKING) → ✅ true
           └─> currentState = INTAKING
           └─> previousState = IDLE

3. SetStateCommand finishes (instant)
   └─> isFinished() returns true

4. CoralIntake starts
   └─> initialize() - resets flags, moves elevator
   └─> execute() - runs intake logic
   └─> isFinished() - waits for coral to be detected

5. CoralIntake finishes
   └─> end() - stops motors, moves elevator to L1

6. Final SetStateCommand runs
   └─> SetStateCommand.initialize()
       └─> StateManager.setState(RobotState.IDLE)
           └─> isValidTransition(INTAKING, IDLE) → ✅ true
           └─> currentState = IDLE
           └─> previousState = INTAKING

7. Sequence Complete!
```

**SmartDashboard shows:**
- `StateManager/CurrentState` = "IDLE"
- `StateManager/PreviousState` = "INTAKING"

---

## 🛡️ Safety Benefits

### **Problem Without State Machine:**
```java
// Driver presses intake button
CoralIntake starts running...

// Operator accidentally presses scoring button
ScoreL4L3L2 starts running...

// CONFLICT! Both commands try to control Effector!
// Elevator might be moving for intake while scoring tries to run
```

### **Solution With State Machine:**
```java
// Driver presses intake button
State = INTAKING
CoralIntake runs...

// Operator presses scoring button
State = INTAKING (still)
ScoreL4L3L2 checks state...
// Can add check: if (state != IDLE) return; // Don't run!
// OR: State transition fails (INTAKING → SCORING not allowed)
```

---

## 💡 How Commands Can Use States

### **Option 1: Check State Before Running**
```java
public class MyCommand extends Command {
    private final StateManager m_stateManager;
    
    @Override
    public void initialize() {
        // Only run if in IDLE state
        if (m_stateManager.getState() != RobotState.IDLE) {
            System.out.println("Cannot run - robot is " + m_stateManager.getState());
            cancel(); // Stop the command
        }
    }
}
```

### **Option 2: Use StateConditionalCommand**
```java
// Only runs if state is INTAKING
StateConditionalCommand.create(
    m_stateManager,
    RobotState.INTAKING,
    new SomeCommand(),
    Commands.print("Not intaking!")
)
```

### **Option 3: Change Behavior Based on State**
```java
@Override
public void execute() {
    if (m_stateManager.isScoring()) {
        // Do scoring-specific behavior
    } else if (m_stateManager.isIntaking()) {
        // Do intake-specific behavior
    }
}
```

---

## 🔗 Integration with Existing Code

### **Your Current Code (Command-Based Only):**
```java
buttonPanel.button(Constants.buttonPanel.coral.IN)
    .onTrue(new CoralIntake(m_elevator, m_effector, m_intake));
```

### **With State Machine (Two Options):**

**Option A: Use State-Aware Wrapper (Recommended)**
```java
buttonPanel.button(Constants.buttonPanel.coral.IN)
    .onTrue(StateAwareCoralIntake.create(
        m_stateManager, m_elevator, m_effector, m_intake
    ));
```

**Option B: Manual State Management**
```java
buttonPanel.button(Constants.buttonPanel.coral.IN)
    .onTrue(Commands.sequence(
        new SetStateCommand(m_stateManager, RobotState.INTAKING),
        new CoralIntake(m_elevator, m_effector, m_intake),
        new SetStateCommand(m_stateManager, RobotState.IDLE)
    ));
```

**Both do the same thing!** Option A is cleaner.

---

## 📊 State Flow Diagram

```
                    ┌─────┐
                    │ IDLE│
                    └──┬──┘
                       │
        ┌──────────────┼──────────────┐
        │              │              │
        ▼              ▼              ▼
   ┌────────┐    ┌─────────┐    ┌─────────┐
   │INTAKING│    │SCORING_L1│    │CLIMBING │
   └───┬────┘    └────┬────┘    └────┬────┘
       │              │              │
       │    ┌─────────┼─────────┐    │
       │    │         │         │    │
       ▼    ▼         ▼         ▼    ▼
   ┌─────────┐  ┌─────────┐  ┌─────────┐
   │SCORING_L2│  │SCORING_L3│  │SCORING_L4│
   └────┬────┘  └────┬────┘  └────┬────┘
        │            │            │
        └────────────┼────────────┘
                     │
                     ▼
                  ┌─────┐
                  │ IDLE│
                  └─────┘
```

**Key Points:**
- All states can return to IDLE
- INTAKING can go to IDLE or scoring states
- Scoring states can switch between each other
- CLIMBING can only go to IDLE

---

## 🎮 Real-World Usage Example

### **Scenario: Driver wants to intake, then score at L4**

**Without State Machine:**
```java
// Driver presses intake
CoralIntake runs...

// Operator presses L4 button (while intake still running)
Elevator moves to L4 (conflicts with intake!)

// Operator presses score button
ScoreL4L3L2 runs (might conflict with intake!)
```

**With State Machine:**
```java
// Driver presses intake
State: IDLE → INTAKING
CoralIntake runs...

// Operator presses L4 button
// Option 1: Check state first
if (stateManager.getState() == RobotState.IDLE) {
    elevator.toPosition(L4);
} else {
    // Wait or queue the command
}

// Option 2: State transition fails
SetStateCommand tries: INTAKING → SCORING_L4
// isValidTransition() returns false
// Transition blocked! State stays INTAKING
```

---

## 🔍 Debugging with States

**SmartDashboard shows:**
- Current state
- Previous state

**Example Debug Session:**
```
StateManager/CurrentState: "INTAKING"
StateManager/PreviousState: "IDLE"

// You know: Robot just started intaking
// If something's wrong, you know the robot thinks it's intaking
```

**In Code:**
```java
// Add logging
System.out.println("Current state: " + m_stateManager.getState());

// Check state in commands
if (m_stateManager.getState() != RobotState.IDLE) {
    Logger.warn("Command called while in state: {}", m_stateManager.getState());
}
```

---

## 🚀 Benefits Summary

1. **Safety**: Prevents conflicting commands
2. **Debugging**: Know what the robot thinks it's doing
3. **Organization**: Clear separation of robot operations
4. **Flexibility**: Can check state anywhere in code
5. **Extensibility**: Easy to add new states or transition rules

---

## 📝 Next Steps

1. **Start using state-aware commands** for new bindings
2. **Add state checks** to existing commands that need them
3. **Monitor state** on SmartDashboard during testing
4. **Adjust transition rules** in `isValidTransition()` as needed
5. **Add new states** if you need them (e.g., `STOWED`, `READY_TO_SCORE`)

---

## ❓ Common Questions

**Q: Do I have to use states for everything?**  
A: No! Existing commands work fine. States are optional but helpful.

**Q: What if I need to bypass state checks?**  
A: Use `MANUAL` state or `forceState()` method.

**Q: Can I have multiple states active?**  
A: No - one state at a time. But you can check previous state if needed.

**Q: How do I add a new state?**  
A: Add it to the `RobotState` enum, then update `isValidTransition()` rules.

---

This state machine gives you the benefits of state-based programming while keeping all the power of the command-based framework!

