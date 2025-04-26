for bag in 2025_Assignment/*.bag; do
  echo "Playing $bag"
  rosbag play "$bag" -l &
done
