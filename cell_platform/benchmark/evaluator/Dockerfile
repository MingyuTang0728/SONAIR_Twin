# The SONAIR evaluator, as a container: the same code, the same numbers,
# wherever it runs. The private targets are mounted at run time, never baked in.
#
#   docker build -t sonair-evaluator -f evaluator/Dockerfile .
#   # score one submission
#   docker run --rm -v $PWD/task_public:/task:ro -v $PWD/task_private:/answers:ro \
#       -v $PWD/my_submission:/sub:ro -v $PWD/out:/out sonair-evaluator \
#       evaluate /sub --task /task --answers /answers --out /out
#   # or run the submission server
#   docker run -p 8090:8090 -v ...:/task:ro -v ...:/answers:ro -v ...:/data \
#       --entrypoint python sonair-evaluator submission_server.py --task /task \
#       --answers /answers --store /data/submissions --keys /data/sonair_keys.json
FROM python:3.12-slim
ENV PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1
RUN pip install --no-cache-dir "numpy>=1.26,<3"
WORKDIR /sonair
COPY sonair_benchmark/ sonair_benchmark/
COPY submission_server.py imu_align.py ur_kin.py ./
RUN useradd -m evaluator && chown -R evaluator /sonair
USER evaluator
ENTRYPOINT ["python", "-m", "sonair_benchmark"]
CMD ["--help"]
