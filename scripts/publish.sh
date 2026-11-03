#!/bin/bash

set -e

cd "$(dirname "$0")/.."

echo "Checking Astro build..."

npm run build

echo "Checking Git status..."

git status --short

echo "Committing changes..."

git add .

git commit -m "${1:-Update robotics blog}"

echo "Pushing to GitHub..."

proxychains git push origin main

echo "Published to GitHub."