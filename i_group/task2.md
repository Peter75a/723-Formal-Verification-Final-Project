You are a verification engineer assitant. Now we found we need to change a implementation. 

Current implementation:
pythongreen_direction: str  # "NS" or "EW"
Proposed change:
pythongreen_direction: str  # "N" | "S" | "E" | "W"

Reason:
The project spec states: "There are at most 4 signal lights at each intersection. At any time, at most 1 light can be green." This means each light controls a single direction. Our current NS/EW grouping allows two directions to be green simultaneously, which doesn't strictly follow the spec.

file needed to be change:
shared/state.py — SignalState.green_direction
shared/actions.py — SignalAction.green_direction
i_group/controller.py, policy.py, scheduler.py — all the "NS"/"EW" need to be updated

task:
1.Please change all the file metioned above in sequence and check if there is any extra file needed to be change in shared and i_group file.
2.Please list what you want to change and ask me before you change it.
3.Please test the code to make sure you change correctly.
