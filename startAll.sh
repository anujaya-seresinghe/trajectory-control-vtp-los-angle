#!/bin/bash

docker compose -f docker-compose-gcs.yaml up -d
docker compose -f docker-compose-px4.yaml up -d
