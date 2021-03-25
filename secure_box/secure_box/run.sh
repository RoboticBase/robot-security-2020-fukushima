#!/bin/bash

openssl enc -d \
        -aes-256-cbc \
        -salt \
        -in ./data/encrypted \
        -out decrypted_file


PYTHON_ENV=$(<decrypted_file)
rm decrypted_file

CMD="${PYTHON_ENV} python3 security_box.py"
eval $CMD
