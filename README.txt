ECE 383/ MEMS 442 / ECE 590 Final exam, Fall 2018
Due 1pm, Dec 12, 2018

==========================================
General instructions
==========================================

You will be assigned a letter from A-F by the instructor / TAs, and will only complete the version of the final assigned to you.  The version-specific instructions will be found in FinalX.ipynb, where X is your assigned letter.

In each final, you are required to perform an implementation and a written report.  80% of your grade will be based on the quality and thoroughness of the report, although a functional implementation is required to write the report.  (To maximize your grade, do not spend all of your time working on perfecting the implementation.  An imperfect implementation tested and described thoroughly will usually score higher than a perfect, unexplained implementation. )

To submit your final, attach both the FinalX.ipynb file and your report (in PDF format) on Sakai. 


==========================================
Bidding process
==========================================

The letter assignment will be conducted via a bidding process to balance preferences and distribution across letter topics.  You will be asked to indicate your top 3 choices, and we will assign students according to these choices.  (In some cases, you may be assigned a letter outside your top choices.)

---------------------------------------------------------------------------
Letter     Topic(s)                   Emphasis
---------------------------------------------------------------------------
A      Motion planning, dynamics   Algorithm implementation
B      Motion planning             Algorithm theory
C      Control, kinematics         Transforms, kinematics, and control
D      Control, manipulation       Controller implementation
E      Perception                  Estimation, numerical methods
F      Perception, manipulation    Optimization, geometry, image processing
---------------------------------------------------------------------------

We will calibrate for the "level of difficulty" across final variations, but your work will still be compared with that of students in your same group.  So, it is beneficial to be assigned a topic that correlates with your areas of strength.


==========================================
Discussion Policy
==========================================

You are NOT in any circumstances allowed to discuss the final with other students that are assigned the same letter.

You MAY discuss the final at a high level with students who are not assigned the same letter, but this should only be at a conceptual level.  THIS IS NOT JOINT WORK. 

Permissible discussions MUST NOT involve details of the programming implementation or writing of the report, although help in debugging general coding bugs may be allowed. A general guideline is that if the coding assistance involves any robotics concepts, not just Python, then it is not allowed.  

For example, OK discussions:
- "How should I find an optimum of this grasp score, since gradient descent would be terrible?":  conceptual question.
- "What would be the most convenient way to implement a state machine for my controller?":  conceptual question.
- "I'm getting an exception assigning a value to this list": pure Python question.
- "How do you set up the Klamp't IK solver to use different joint limits?": question about a Python API rather than robotics concepts.
- "How can I show a configuration on the visualization for debugging?":  question about a Python API rather than robotics concepts.
- "How do I get the depth at a pixel?": question about a Python API rather than robotics concepts.
- "Can I see how you implemented Lab 4?": tangential, conceptual discussion.

NOT OK discussions:
- "My control loop isn't doing the right thing, how should I fix it?": question about the behavior of a control loop.
- "The IK solver is making the robot go to a weird configuration, how can I fix it?": question about coordinate transforms and behavior of a robotics component.



The instructor / TA will be able to provide further assistance in office hours or by appointment.


==========================================
Report
==========================================

The report may be prepared in whatever program you wish (e.g., Word, LaTeX, Jupyter notebook).  It should be divided into sections roughly corresponding to the prompts in FinalX.ipynb.  Include equations, images, and figures as necessary to explain your design, implementation, and testing.  Typical reports will be roughly between 4-10 pages in length. 

The report will be graded on the basis of correctness, thoroughness, and clarity.  Be frank when discussing of the limitations of your implementation.  An incorrect implementation, described and tested honestly, will be scored more highly than a report written pretending that the implementation works.


==========================================
Implementation
==========================================

We will run your FinalX.ipynb file to examine the behavior of your implementation.  If your file requires special care to run (e.g., we should skip broken or system-dependent cells for grading), place these instructions a cell at the top of the notebook, clearly titled "Running This Notebook".


