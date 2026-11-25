# SONAIR starter kit

1. Download and unzip `sonair_task_v2_public.zip`. `TASK.md` inside says exactly what each file holds.
2. Make a first submission:

   ```
   python make_submission.py --task sonair_task_v2_public --out my_submission
   ```

3. Check it before you send it. This uses only the public package, so a submission that passes here is not rejected for its form:

   ```
   python -m sonair_benchmark validate my_submission --task sonair_task_v2_public
   ```

4. Zip `my_submission/` and submit it with your API key:

   ```
   curl -H "Authorization: Bearer <your key>" -H "Content-Type: application/zip" --data-binary @my_submission.zip https://<sonair server>/api/submit
   # -> {"id": ..., "status_url": "/api/status/<id>"}; poll it, then open /scorecard/<id>
   ```

   You get back a scorecard: the headline score (Effort Tail GCR-p95), both tracks, robustness under degraded input, calibration if you gave `sigma`, and every held-out cell.

**What to change:** `fit` and `predict` in `make_submission.py`. The file format must stay the same. You can predict a per-sample `sigma` for each channel; you will then be scored on calibration as well.

**Optional vision track:** add `"vis": [[x, y, z], ...]` (m, robot base frame, on the input timestamps): the tool position the tool camera will see. It is scored at the camera's frame times against rigid kinematics of the commanded joints, using the `tcp_offset` in each run's header. Train runs with camera data have a `<run>.vis.json` beside them.
