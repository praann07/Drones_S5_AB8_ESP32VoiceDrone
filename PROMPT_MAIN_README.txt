PROMPT FOR CLAUDE CODE (terminal CLI)
=====================================
How to use:
1. Put the three intro files in one empty folder, for example:
     INTRO_DRONE_MUJOCO.md
     INTRO_DRONE_GAZEBO.md
     INTRO_DRONE_PYBULLET.md      (the tittu-09-voice-drone branch)
   (If your friends named theirs differently, keep their real names and edit the three names in the prompt below.)
2. Open a terminal in that folder and start Claude Code:  claude
3. Paste everything between the two lines marked ===== PROMPT START / END =====.
4. It will write README.md in that folder. Read it, then copy it to the main branch yourself.

===== PROMPT START =====

You are a senior technical writer and robotics/controls engineer. This folder contains three introduction files, one per branch of the same team repository:

  1. INTRO_DRONE_MUJOCO.md   branch `drone_mujoco`          MuJoCo, SINDy-identified attitude loop, voice demo on a campus model
  2. INTRO_DRONE_GAZEBO.md   branch `drone_gazebo`          PX4 + Gazebo voice-controlled quadrotor, SINDy/DMDc identification
  3. INTRO_DRONE_PYBULLET.md branch `tittu-09-voice-drone`  PyBullet city simulation, speech-driven command parser, obstacle handling

Read all three files completely first. Then write ONE file named README.md in this folder: the front page of the `main` branch of the repository. It must introduce the whole team project, present all three attempts faithfully, and state clearly how they differ. A non-expert and a mathematics professor must both be able to read it. Do not commit, push, or modify any other file.

CONTEXT
- Repository: https://github.com/praann07/Drones_S5_AB8_ESP32VoiceDrone
- Course: Data Driven Control of Drones, B.Tech AI Engineering, School of Artificial Intelligence, Amrita Vishwa Vidyapeetham, Coimbatore. The reader is a professor reviewing the project.
- Team (keep exactly):
    Mamidala Som Praneeth Babu  CB.SC.U4AIE24029  cb.sc.u4aie24029@cb.students.amrita.edu
    C. Karthik                  CB.SC.U4AIE24107  cb.sc.u4aie24107@cb.students.amrita.edu
    Ch. Virinchi                CB.SC.U4AIE24158  cb.sc.u4aie24158@cb.students.amrita.edu
    K. Jatin Chandra Gupta      CB.SC.U4AIE24162  cb.sc.u4aie24162@cb.students.amrita.edu
- The `main` branch used to hold the team's first attempt, an ESP32-style Python simulator. Mention it in ONE short line as the origin of the project, and do not carry over its content. The logo file `amrita_logo.png` already exists at the root of main; use it with a relative link at the top.

WORKFLOW (do these in order, silently, then output only the file)
1. Read the three files fully.
2. Build an inventory for each attempt: goal, simulator, how the drone is modelled, controller, whether and how system identification (SINDy, DMDc, others) is used, voice or speech method, environment or map, headline measured results, tests, limitations, how to run, images/GIFs/videos, branch name.
3. Build a comparison from the inventory. Every cell must come from the files. If a file does not say something, write "not stated" in the cell. Never guess.
4. Write README.md using the structure below.
5. Run the checklist at the end on your own output, and fix every failure.
6. Finish with a short message in the terminal listing (a) every conflict or gap you found and how you handled it, (b) anything you could not verify. Keep it under 15 lines.

REQUIRED STRUCTURE FOR README.md
1. Centered logo, then a centered title for the whole project, a one-line pitch, the course and university line, and a navigation bar linking to the sections below.
2. Hero visual: choose the single strongest GIF from the three intro files, with a caption naming the branch it comes from.
3. "The project in one page": what problem the team set out to solve, the shared goal (a voice-controlled quadrotor in simulation, built around data-driven identification and control), and why three attempts exist. Two short paragraphs in plain language.
4. "The three attempts at a glance": ONE comparison table with one column per attempt and one row per aspect: simulator, vehicle model, system identification, controller, voice interface, environment, headline results, tests, main limitation, branch link. Each cell short.
5. "How the attempts differ, and why": a plain-language discussion of the real differences in approach, for example what the identified model is used for, how realistic the physics is, how speech is handled, how obstacles or maps are treated. Be fair to each attempt. State strengths and weaknesses taken from the intro files only. Do not rank the attempts and do not use marketing words.
6. Three full attempt sections, in this order: Attempt 1 `drone_mujoco`, Attempt 2 `drone_gazebo`, Attempt 3 `tittu-09-voice-drone`. Each section keeps the content of its intro file (headings, equations, tables, measured numbers, images), edited only to remove repetition and to keep heading levels consistent (each attempt is an H2, its parts are H3). Keep the equations in code blocks with symbols defined at first use.
7. "What we learned across the three attempts": the lessons that appear in two or more attempts, then lessons unique to one attempt, each tagged with the attempt it came from.
8. "Honest limits": the combined, de-duplicated list of what was not done, with the attempt each point applies to.
9. "Run it yourself": a table with one row per attempt: branch, one-line command or launcher, and what you will see. Take commands only from the intro files.
10. "Repository guide": which branch holds what, with a link to each branch (https://github.com/praann07/Drones_S5_AB8_ESP32VoiceDrone/tree/<branch>).
11. "Team": the table above, then the references that appear in the intro files, de-duplicated.

HARD RULES
- Do not invent anything: no numbers, results, features, file names, links, citations, or team contributions. If something is missing, say "not stated" or leave it out.
- Keep every measured number exactly as written in its intro file. If two files give different numbers for the same quantity, keep each in its own attempt section and list the conflict in the final terminal message.
- Keep all stated limitations. Do not soften them. Do not add claims such as "best" or "state of the art".
- Facts verified for `drone_mujoco`, which must stay exact: SINDy has 26 candidate terms in each of 6 equations, which is 156 coefficients, and 44 are non-zero (never write "156 terms"); held-out R-squared 0.9946; one-step NRMSE 0.0786; DMDc R-squared 0.9926 near hover, and a one-step NRMSE of about 0.70 outside hover (an NRMSE, not an R-squared); 0 of 7 eleven-second rollouts diverged; braking from 1.0 m/s takes 0.92 s over 0.50 m; 167 tests; no Kalman filter and no controllability or reachability analysis.
- All image, GIF and video links must be ABSOLUTE https://raw.githubusercontent.com/praann07/Drones_S5_AB8_ESP32VoiceDrone/<branch>/<path> URLs, exactly as they appear in the intro files. Do not convert them to relative paths and do not add external links that are not in the sources. The only relative link allowed is amrita_logo.png.
- GitHub-flavoured Markdown only. Plain language first, mathematics second, code location third. Short sentences. No filler, no flattery, no emojis in headings.
- Length: as long as needed to lose nothing, but no fact may appear twice except in the comparison table, which may repeat headline numbers once.

FINAL CHECKLIST (verify before finishing)
[ ] All three attempts are present, in the order given, and each keeps its equations, tables, measured numbers and images.
[ ] The comparison table has no invented cells; gaps say "not stated".
[ ] The differences section is fair, factual and contains no ranking.
[ ] No conflicting numbers were merged silently; conflicts are listed in the terminal message.
[ ] All four team members, roll numbers and emails are present exactly.
[ ] All image links are absolute raw.githubusercontent.com URLs, and the logo link is relative.
[ ] A reader with no control-theory background can follow sections 3 to 5.
[ ] Heading levels are consistent and the navigation links match real headings.
[ ] README.md was written in this folder; nothing was committed or pushed.

===== PROMPT END =====
