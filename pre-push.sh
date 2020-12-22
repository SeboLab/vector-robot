#!/bin/bash
echo "Running pre-push linters and checks..."
black -t py38 -q .
flake8 --max-line-length=96 --show-source --exclude .git,__pycache__
pydocstyle --ignore=D102,D103,D105,D107,D213,D203,D101,D100
pylint ./nodes/*.py --good-names=k,v,i,tz,ts,w,h --disable=C0116,W0105
pylint ./sample_nodes --good-names=k,v,i,tz,ts,w,h,op,a,b,e --disable=C0116,W0105
