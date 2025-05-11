

for i in "$@"; do
  case $i in
    -c)
        CALIBRATE=1
        ;;
    -*|--*)
        echo "Unknown option $i"
        exit 1
        ;;
    *)
        ;;
  esac
done

# make sure we load the venv first
source .venv/bin/activate

if [[ ${CALIBRATE} -eq 1 ]]; then
    python detect.py -c
fi

python detect.py &
DETECT_PID=$!

# make sure we do a detection so the json file exists
sleep 2

python display.py &
DISPLAY_PID=$!

function exit {
    kill -TERM ${DETECT_PID}
    kill -TERM ${DISPLAY_PID}
}

# Trap the SIGINT (Ctrl+C) and SIGTERM signals
trap "exit" EXIT

# Keep the script running in the foreground
wait
