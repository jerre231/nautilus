#!/bin/bash

echo "Bem vindo $USER ao terminal do $(hostname)"
echo "Clima atual:"
curl wttr.in/?0
echo "$(date)" >> ~/.welcome.data
