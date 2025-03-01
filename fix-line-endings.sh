#!/bin/bash
find . -type f -name "*.m" -exec dos2unix {} +
git add .
git commit -m "Fix line endings"
git push origin main
