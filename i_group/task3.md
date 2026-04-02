You are a verification engineer, and we are currently in the final review stage of the traffic system project.

I have found an issue in the i-group code: it may generate signals for nonexistent directions.

Your tasks are

1. Go through the entire codebase and identify the root cause of this issue.

Before making any revisions, clearly explain:
- why this issue happens,
- which files / functions are involved.

Then propose a fix plan with the following strategy:

Fix Strategy:
- In policy.py, introduce a helper function to retrieve the valid directions for each intersection.This helper should be implemented inside the SignalPolicy class, and placed near existing helper functions such as _count_approaching() for clarity and maintainability.The purpose of this helper is to ensure that all downstream logic (e.g., pressure calculation and action generation) only considers topology-valid directions.This enforces topology constraints at the source of action generation, preventing invalid SignalAction outputs.Please use topoloy.py function get_intersection

- In addition, extend test.py to include validation checks:
  - Verify that no intersection generates signals for directions that do not exist.
  - Treat any such case as a test failure.

Important:
- Do NOT modify the shared/ folder.
- Do NOT implement the fix before presenting the analysis and plan.
- The solution must ensure that invalid directions are impossible, not just unlikely.

2. Review the rest of the code and identify any other potential issues, inconsistencies, edge-case bugs, or logic violations.
   I will go through those findings one by one with you before deciding what to revise.

Important requirements:
- Do not modify anything before first giving me your analysis and proposed fix plan.
- Pay special attention to whether each intersection only produces valid signal directions according to its actual topology.
- Check for hidden issues related to invalid actions, boundary cases, and conflicts with the project requirements.
- Be conservative and thorough, since this is the final review stage and we cannot afford overlooked errors.