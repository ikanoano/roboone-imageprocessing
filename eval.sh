echo "run imtest & plot"
set -x
make EVAL=1 imtest
timeout 10 ./imtest | sed 's/[\{\}]//g' | sed 's/ ,/,/g' > /tmp/imtest_data
tail -n +5 /tmp/imtest_data | ./plot.awk > /tmp/imtest_data.g
gnuplot /tmp/imtest_data.g -p
