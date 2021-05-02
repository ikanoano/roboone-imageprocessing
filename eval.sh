echo "run imtest & plot"
set -x
make EVAL=1 imtest \
  && timeout 300 ./imtest \
  | sed -u 's/[\{\}]//g' \
  | sed -u 's/ ,/,/g' \
  | gawk -f ./plot.awk \
  | tee /tmp/imtest_data.g \
  | gnuplot -p
