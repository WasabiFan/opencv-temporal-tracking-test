#!/bin/bash

TARGET_HOST=jetson-488-alpha
TARGET_USER_NAME=ubuntu
SOURCE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
DEST_DIR=/home/$TARGET_USER_NAME/opencv-temporal-tracking-test/

# TODO: Add logic to do a production-ready build, and maybe configure the service 

COMPILE_ONLY=false
for i in "$@"
do
    case $i in
        -c|--compile-only)
            COMPILE_ONLY=true
            shift
        ;;
        *)
            # unknown option
        ;;
    esac
done

# TODO: Ping to confirm host exists
#DATE=$(date)
#echo "$DATE"
#ssh $TARGET_USER_NAME@$TARGET_HOST sudo date --set "$DATE"

#exit 0

ssh $TARGET_USER_NAME@$TARGET_HOST mkdir -p "$DEST_DIR"
rsync -azvv -e ssh --exclude=.git --exclude=build --exclude='Visual Studio Solution' --exclude='*~' --exclude='.gitignore' $SOURCE_DIR/ $TARGET_USER_NAME@$TARGET_HOST:$DEST_DIR

ssh $TARGET_USER_NAME@$TARGET_HOST -X << EOF
    if mkdir -p "$DEST_DIR/build" &&
    cd "$DEST_DIR/build" &&
    cmake ../ &&
    make; then
        if ! [ "$COMPILE_ONLY" = true ]; then
            ./temporal-tracking-test
        fi
    else
        echo "Build failed!"
    fi
EOF
