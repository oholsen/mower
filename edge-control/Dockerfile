FROM python:3.8-slim-buster as requirements

RUN python -m pip install --upgrade pip
RUN python -m pip install --upgrade poetry
RUN poetry config virtualenvs.create false

WORKDIR /app

# FIXME: need non-release fix for asyncio-mqtt
COPY pyproject.toml ./
COPY poetry.lock ./
RUN poetry export -E jackal --without-hashes -f requirements.txt --output requirements.txt

FROM python:3.8-slim-buster
RUN python -m pip install --upgrade pip
WORKDIR /app
COPY --from=requirements /app/requirements.txt .
RUN python -m pip install --force-reinstall -r requirements.txt
ADD edge_control ./edge_control

ENTRYPOINT [ "python3", "-m", "edge_control.mqtt" ]
