cd ../../

pwd

# Prefixes required for compilation
patterns=("BHI26*" "BMA42*")

examples=()
for pattern in "${patterns[@]}"; do
    while IFS= read -r -d '' dir; do
        dir_name=$(basename "$dir")
        examples+=("$dir_name")
    done < <(find examples/ -maxdepth 1 -type d -name "$pattern" -print0)
done

readarray -t examples < <(printf '%s\n' "${examples[@]}" | sort -u)

envs=(
    "esp32dev_arduino"
    "nrf52840_arduino"
    "rp2040_arduino"
    )

echo "Cleaning..."
pio run -t clean > /dev/null 2>&1

total=$(( ${#envs[@]} * ${#examples[@]} ))
current=0

echo "Found ${#examples[@]} examples to build"

for env in "${envs[@]}"
do
    for value in "${examples[@]}"
    do
        current=$((current + 1))
        
        if [ -f "$value/.skip."$env ];then
            echo "[$current/$total] Skipped: $value [$env]"
            continue
        fi

        export PLATFORMIO_SRC_DIR="examples/$value"
        echo -n "[$current/$total] Building: $value [$env] ... "
        
        output=$(pio run -e "$env" 2>&1)
        if [ $? -eq 0 ]; then
            echo "OK"
        else
            echo "FAILED"
            echo "Error output:"
            echo "$output"
            exit -1
        fi
    done
done

echo "All builds completed successfully!"