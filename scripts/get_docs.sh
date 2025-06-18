#!/bin/bash

# --- Settings ---
DOCS_DIR="docs"          # Docs path
PORT=8000                # Local server port

## --- Check folder ---
#if [ ! -d "$DOCS_DIR" ]; then
# echo "Error: Documentation folder '$DOCS_DIR' not found!"
#  exit 1
#fi

#cd "$DOCS_DIR" || exit 1

## --- Build HTML ---
echo "Building html..."
#if ! make html; then
#  echo "Error during bilding!"
#  exit 1
#fi

## --- Start server ---
echo "Starting server at http://localhost:$PORT"
echo "Press Ctrl+C to stop"

python3 -m http.server --directory ./docs/_build/html "$PORT"
