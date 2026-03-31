# 🚦 Final Project – Simplified Traffic System

Hi everyone,
This is our final project. We will design a simplified traffic system together.

---

## ⚠️ Team Rules (Must Follow)

### 🔹 Branching Rules

* **Do NOT modify the `main` branch directly.**
* Before making any changes, you must **create your own branch**.
* Please follow the workflow provided below.

---

### 🔹 Project Structure & Responsibilities

* Each group should only work within their own directories:

  * `i_group/`
  * `v_group/`
* We use **function-based interfaces** through a single `simulator`.

---

### 🔹 Shared Folder (Strict Contract)

* The `shared/` folder defines the **interface between groups**:

  * `v_group` → outputs `CarAction`
  * `i_group` → outputs `SignalAction`

* 🚫 **DO NOT modify any files in `shared/`**, including:

  * `state.py`
  * `enums.py`
  * `actions.py`
  * `topology.py`

* Any changes to shared files require:

  1. Informing the entire team
  2. Explaining the change and its impact
  3. Getting agreement from everyone

---

### 🔹 Simulation Rules

* Slot range: `0 → 29`
* Crossing: occurs from **slot 29 → intersection**
* Must check **red light before entering intersection**
* Cars must **not move if another car is visible ahead**
* **At most ONE car per intersection per step**

---

## 🚧 Phase A Requirements

* Each group works **only within their own folder** (`i_group` / `v_group`)
* Ensure your code **runs successfully** and **passes all required tests**.
* All implementations must **strictly follow the shared interfaces and variable definitions**

---

## 🔁 Git Workflow (Reference)

```bash
# 1. Start from latest main
git checkout main
git pull origin main

# 2. Create your own branch
git checkout -b your-branch

# --- develop your feature ---

# 3. Sync with latest main before continuing
git checkout main
git pull origin main
git checkout your-branch
git merge main

# 4. Stage, commit, and push
git add .
git commit -m "feat: describe your change"
git push

# 5. Open a Pull Request (PR) on GitHub
# → Request review → Merge after approval

# 6. Update your local main after merge
git checkout main
git pull origin main
```

---

## ✅ Key Principles

* Develop in **branches**
* Integrate in **main via PR**
* Keep `shared/` **consistent and untouched**
* Ensure **compatibility between groups**
