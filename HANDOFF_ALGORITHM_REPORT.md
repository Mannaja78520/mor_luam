# HANDOFF — algorithm_sim + report (2026-11-04)

## Finished
- `algorithm_sim/`: Detour Steer docs (algorithm.pdf/tex/html, explain_th.md, HW04_README_th.md),
  Python simulators (no Docker), robot-result figures. Tests pass:
  `python -m unittest test_current_robot`, `python detour_steer.py --no-bench` (T1-T4 PASS).
- `algorithm_sim/README.md`: run without Docker; Docker recommended for firmware/ROS 2, install steps.
- Root `README.md`: section linking `algorithm_sim/` with the Docker-recommended note.
- `algorithm.pdf` now ends with its own page holding only the Git link (also copied to the homework folder); report.pdf Appendix A Git page is alone, code listings start on the next page.
- `algorithm_sim/docs/report.pdf` (9 pages) = algorithm.tex + Appendix A: Git URL, clone/run
  commands, Python planner listing (detour_steer.py 241-299), DetourSteer.h/.cpp listings.
  Build: `cd algorithm_sim/docs && pdflatex report.tex` (twice). No YouTube link (user: not needed).

## In progress
- None.

## Next
- If code changes, rebuild report.pdf (listings are read from the files at build time;
  Python line range 241-299 is fixed — re-check after editing detour_steer.py).
- Homework copy in E:\271401\271401_pee_aut_homework is still uncommitted (separate repo).
