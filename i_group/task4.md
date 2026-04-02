You are a verification engineer, and our project is in the final review stage. Now we want to make sure our tesk_i_group.py is perfect.


## 🛠️ Tasks

### ✅ Task 1 — Max-Pressure Selection

**Goal:**
Verify that the controller selects the direction with the highest vehicle count.

**Test Strategy:**

* Use a fully connected intersection (`I11`)
* Assign more vehicles to one direction
* Ensure scheduler allows switching

**Expected Result:**

* The chosen phase equals the direction with maximum vehicles

---

### ✅ Task 2 — Round-Robin Tiebreaker

**Goal:**
Verify fairness when all directions have equal pressure.

**Test Strategy:**

* Assign equal vehicles to all directions
* Trigger forced switching (`max_green`)
* Run multiple steps

**Expected Result:**

* Selected direction rotates across steps
* No direction is repeatedly selected

---

### ✅ Task 3 — Min-Green Constraint

**Goal:**
Ensure the system does NOT switch too early.

**Test Strategy:**

* Set timer < `min_green`
* Provide strong pressure in another direction

**Expected Result:**

* Signal **must remain unchanged**

---

### ✅ Task 4 — Max-Green Constraint

**Goal:**
Ensure the system eventually switches.

**Test Strategy:**

* Set timer ≥ `max_green`
* No pressure difference required

Task 5 please go though all the i_group folder files and check if there is any case no be covered. Also make sure there is no bug in this folder.