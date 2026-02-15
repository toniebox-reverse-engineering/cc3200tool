#!/bin/bash
export PYTHONPATH=$PYTHONPATH:$(pwd)/cc3200tool
python3 tests/mock_esp_gateway.py
