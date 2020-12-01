#!/bin/bash
echo "Running pre-push linters and checks..."
black -t py38 -q .
flake8 --max-line-length=96 --show-source --exclude .git,__pycache__
pydocstyle --ignore=D102,D103,D105,D107,D213,D203,D101,D100
pylint ./**/*.py --good-names=k,v,ip,i,tz,ts --disable=C0116
