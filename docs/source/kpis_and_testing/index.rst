Unit Testing KPIs for TIAGo Cooking Assistant
=============================================

Introduction
------------
This document defines the Key Performance Indicators (KPIs) for unit testing the cognitive architecture of the TIAGo Cooking Assistant. These KPIs verify component behavior, validate system interactions, and assess real-world scenario handling capabilities.

1. Recipe Tracking KPIs
-----------------------

+------------------------------+---------------------------------------------------+-----------------------------------------+
| KPI                          | Description                                       | Success Criteria                        |
+==============================+===================================================+=========================================+
| Recipe Parsing Reliability   | Robustness in parsing JSON recipes                | 100% success for valid JSON; graceful   |
|                              |                                                   | rejection of invalid inputs             |
+------------------------------+---------------------------------------------------+-----------------------------------------+
| Correct Step Recording       | Updates execution history after step completion   | 100% of completed actions logged        |
+------------------------------+---------------------------------------------------+-----------------------------------------+
| Step Transition Accuracy     | Correct step transitions after IDLE period        | 100% correct increments after 3s IDLE   |
+------------------------------+---------------------------------------------------+-----------------------------------------+
| Step Order Integrity         | Sequence matches predefined recipe                | 100% sequence consistency               |
+------------------------------+---------------------------------------------------+-----------------------------------------+
| Missing Step Detection       | Detects skipped steps                             | 95%+ detection accuracy                 |
+------------------------------+---------------------------------------------------+-----------------------------------------+
| Recipe Representation        | Retrieves correct current/next step               | 100% correct retrieval                  |
+------------------------------+---------------------------------------------------+-----------------------------------------+

2. Action Planning KPIs
-----------------------

+------------------------------+---------------------------------------------------+-----------------------------------------+
| KPI                          | Description                                      | Success Criteria                         |
+==============================+===================================================+=========================================+
| Action Sequence Mapping      | Generates correct action sequences               | 100% match with predefined mappings      |
+------------------------------+---------------------------------------------------+-----------------------------------------+
| Command Insertion Correctness| Inserts commands per urgency rules               | ≥95% follow urgency policy               |
+------------------------------+---------------------------------------------------+-----------------------------------------+
| Planning Responsiveness      | Time to generate action sequences                | <200ms from step reception               |
+------------------------------+---------------------------------------------------+-----------------------------------------+
| Interruption Handling        | Behavior during sequence interruptions           | 100% correct restructuring               |
+------------------------------+---------------------------------------------------+-----------------------------------------+
| State Synchronization        | Internal index matches controller feedback       | 0% desync                                |
+------------------------------+---------------------------------------------------+-----------------------------------------+
| Service Availability Handling| Behavior during service unavailability           | Graceful retries without crashes         |
+------------------------------+---------------------------------------------------+-----------------------------------------+
| Next Action Correctness      | Chooses correct next action                      | ≥95% correct decisions                   |
+------------------------------+---------------------------------------------------+-----------------------------------------+
| Dynamic Adjustment Capability| Adjusts plans for unexpected events              | Correct in ≥90% of cases                 |
+------------------------------+---------------------------------------------------+-----------------------------------------+
| Plan Consistency             | Logical coherence after adjustments              | 100% verification                        |
+------------------------------+---------------------------------------------------+-----------------------------------------+
| Response Time                | Time to generate next action                     | ≤500ms per decision                      |
+------------------------------+---------------------------------------------------+-----------------------------------------+

3. Human Command Manager KPIs
-----------------------------

+------------------------------+---------------------------------------------------+-----------------------------------------+
| KPI                          | Description                                       | Success Criteria                        |
+==============================+===================================================+=========================================+
| Command Recognition Accuracy |  Identifies verbal commands                       | ≥90% accuracy                           |
+------------------------------+---------------------------------------------------+-----------------------------------------+
| Conflict Detection Rate      |  Identifies command-recipe conflicts              | ≥95% detection                          |
+------------------------------+---------------------------------------------------+-----------------------------------------+
| Correct Decision on Conflicts|  Appropriately accepts/rejects commands           | ≥95% correct decisions                  |
+------------------------------+---------------------------------------------------+-----------------------------------------+
| Conflict Handling Response   |  Time to resolve conflicts                        | ≤800ms average                          |
+------------------------------+---------------------------------------------------+-----------------------------------------+
| Non-Urgent Ratio Calculation |  Calculates urgency ratios accurately             | 100% accuracy                           |
+------------------------------+---------------------------------------------------+-----------------------------------------+
| Response Latency             |  Time to process verbal commands                  | <100ms from receipt                     |
+------------------------------+---------------------------------------------------+-----------------------------------------+
| Sequence Update Handling     |  Robustness during concurrent updates             | 0% corruption or data loss              |
+------------------------------+---------------------------------------------------+-----------------------------------------+

4. Cross-Component KPIs
------------------------

+------------------------------+-------------------+---------------------+-----------------------------------------+
| KPI                          | Metric            | Testing Approach    | Success Criteria                        |
+==============================+===================+=====================+=========================================+
| End-to-End Decision Latency  | <500ms from       | Full system testing | Response within latency constraint      |
|                              | command to action |                     |                                         |
+------------------------------+-------------------+---------------------+-----------------------------------------+
| Error Recovery               | 100% recovery     | Inject invalid      | Maintains safe operation                |
|                              | from malformed    | commands            |                                         |
|                              | commands          |                     |                                         |
+------------------------------+-------------------+---------------------+-----------------------------------------+
| Concurrency Performance      | <5% drop under    | Stress testing      | Maintains responsiveness                |
|                              | 10x load          |                     |                                         |
+------------------------------+-------------------+---------------------+-----------------------------------------+
| Memory Leak Resistance       | <1% memory        | Long-term stress    | Stable memory usage                     |
|                              | increase after    | testing             |                                         |
|                              | 1000 cycles       |                     |                                         |
+------------------------------+-------------------+---------------------+-----------------------------------------+

Conclusion
----------
These KPIs provide a structured methodology to assess the performance and correctness of the cognitive architecture. Successful validation establishes a solid foundation for integration testing and real-world deployment.
