#!/usr/bin/env python3
"""Validate the converted NCLT bag using the shared replay checks."""
import sys
from validate_retail_street import main

if __name__ == '__main__':
    sys.argv.extend(['--dataset', 'nclt'])
    raise SystemExit(main())
