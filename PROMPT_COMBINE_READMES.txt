You are a senior technical writer and robotics/controls engineer. I am giving you THREE README-style Markdown files about ONE project. Merge them into ONE single, cumulative, polished README.md that is the best possible front page for this project. Nothing important may be lost, nothing may be repeated, and nothing may be invented.

PROJECT CONTEXT (so you can judge what matters)
A voice-controlled quadrotor (drone) simulated in MuJoCo on a 1:10 scale model of the Amrita Coimbatore campus. The drone's rotational dynamics are learned from simulated flight data with SINDy (and compared with DMDc). The inner attitude controller's gains are calculated from the learned model; the outer position loop is standard physics. Speech (Vosk, offline) or typed text commands the drone ("forward", "stop", "go to central library"). It is a course project for "Data Driven Control of Drones", presented to a professor, so it must read as rigorous, honest and complete.

INPUT
Below this prompt I will paste three files. Treat all three as source material. They overlap and may disagree.

YOUR JOB, IN ORDER
1. Read all three files fully before writing anything.
2. Build a hidden inventory: every fact, number, table, equation, image/GIF/video link, command, file path, name and caveat that appears in any file. Tag each with the file it came from.
3. Find overlaps (same content said twice) and conflicts (different numbers or claims for the same thing).
4. Write one merged README.md following the structure below.
5. Run the checklist at the end on your own output and fix anything that fails.
6. Output ONLY the final README.md inside one Markdown code block, then a short "Conflicts and decisions" list after it (max 10 lines).

RULES FOR CONFLICTS
- Never silently pick. If two files disagree on a number or claim, prefer the figure that comes with a source file, script or measurement; otherwise prefer the more conservative claim; and list it in "Conflicts and decisions".
- Known facts you must keep exactly (they were verified against the code):
  * SINDy has 26 candidate terms in each of 6 equations = 156 coefficients; 44 are non-zero. Never say "156 terms".
  * Inner-loop design settling time is 0.5 s with damping ratio 0.7. Outer loop: 2.0 s, damping 1.0.
  * SINDy held-out R-squared 0.9946, one-step NRMSE 0.0786. DMDc near hover R-squared 0.9926, NRMSE 0.0825, and a one-step NRMSE of about 0.70 outside hover (not an R-squared).
  * Rollouts: SINDy 0 of 7 diverged over 11 s.
  * Braking from 1.0 m/s: rest in 0.92 s over 0.50 m.
  * 157 automated tests; 24/24 voice trials; 5/5 chained; 67/67 gauntlet.
  * There is NO Kalman filter and NO controllability/reachability analysis. Do not claim them. List them under future work.
  * DMDc is valid near hover only. Results are simulation only. Building heights are estimated.
- Do not invent numbers, results, features, file names, links, citations or team members. If something is missing, leave it out; do not guess.
- Keep the honest limits. Do not soften them or remove them.

REQUIRED STRUCTURE (keep this order; merge, do not duplicate sections)
1. Centered title, one-line pitch, course and university line, badges (Python, MuJoCo, SINDy+DMDc, SINDy R-squared, tests, platform), a one-line navigation bar linking to the main sections.
2. Hero visual (the demo GIF linking to the MP4) with one caption line.
3. "At a glance" table (what was learned, how it flies, how it is commanded, how well it works, how it was stressed).
4. Objective and the one design decision (only the inner loop is learned, and why).
5. System overview diagram (ASCII or Mermaid) of the whole chain: voice -> command -> route/trajectory -> outer loop -> inner loop -> mixer -> MuJoCo -> sensors -> back.
6. Pipeline stage table (A excitation, B identification, C validation, D control, E voice demo) with key result and folder for each.
7. "How it works, in plain words" for a newcomer: how a quadrotor flies, what SINDy and DMDc do, what the controller does, how voice and path planning work. Use short analogies. Define each term the first time it appears.
8. Method in detail, with the equations in a table or code blocks and the file each one lives in: SINDy, STLSQ, DMDc, quaternion error and control law, gain formulas, mixer, outer PD plus gated integral, interpolation (A*, cubic smoothing spline, quintic Bezier), stopping distance v^2/(2a), voice classifier, safety clip.
9. The campus world: sources (OpenStreetMap, Esri satellite), per-building alignment, what is physics-inert.
10. Results: braking, identification and validation, control, voice trials and gauntlet, autonomy, robustness. Keep every image link.
11. "What building this taught us" (numbered lessons) and an assumptions table.
12. Honest limits, then future work.
13. How to run (RUN.bat menu, what to say, reproduce commands, test table).
14. Repository layout tree.
15. A short "For a newcomer: reading order" pointing to deeper docs.
16. Team table with names and roll numbers, then branches, then credits and references.

STYLE
- GitHub-flavoured Markdown. Plain language first, maths second, code location third.
- Short sentences. No filler, no marketing words, no flattery.
- Tables for comparisons, numbered lists for steps, code blocks for equations and commands.
- Every table and list must add information. Remove anything that only repeats another section.
- Keep relative links and image paths exactly as they appear in the sources. Do not add external links you were not given.
- Target length: as long as needed to lose nothing, but no section may say the same thing twice.

FINAL CHECKLIST (verify before answering)
[ ] Every number, equation, image, GIF, video, table and command from all three files is present once.
[ ] No contradictory numbers remain; each conflict is listed after the README.
[ ] No invented content; no Kalman filter or controllability claim.
[ ] All relative links and image paths are unchanged from the sources.
[ ] All four team members and roll numbers present.
[ ] A reader with no control-theory background can follow sections 4 to 8.
[ ] Headings are consistent and the navigation bar links match real headings.
[ ] Output is a single Markdown code block followed by the short conflicts list.

Here are the three files:

=== FILE 1 ===
(paste file 1)

=== FILE 2 ===
(paste file 2)

=== FILE 3 ===
(paste file 3)
