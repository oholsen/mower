set -e
poetry run mypy --install-types --non-interactive edge_control tests
poetry run black . && poetry run isort .
poetry run pytest
poetry run python -m tests.simulation Mowing --sim --config config/mowing
# TODO: also a short async test/mission (without --sim)
