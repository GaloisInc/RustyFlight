# Error checking script
#
# Kai Brooks
# github.com/kaibrooks

# counts and outputs the number of warnings/errors in each of the logs in
# c2rust-logs, saves them as text files

# change the output filename here
full_log="full_errors.txt"
count_log="count_errors.txt"


rm "$count_log"
for file in ./c2rust-logs/*
do
  echo 'Checking ' $file
  echo $file >> "$count_log"
  echo 'Errors:' | tee -a "$count_log"
  grep -i -c 'error:' "$file" | tee -a "$count_log"
  echo 'Warnings:' | tee -a "$count_log"
  grep -i -c 'warning:' "$file" | tee -a "$count_log"
  echo '\n' >> "$count_log"
done

echo 'Saved results to ' "$count_log"

rm "$full_log"
for file in ./c2rust-logs/*
do
  echo 'Checking ' $file
  echo '************************************************' >> "$full_log"
  echo $file >> "$full_log"
  echo '************************************************' >> "$full_log"
  echo 'Errors:' | tee -a "$full_log"
  echo '************************************************' >> "$full_log"
  grep -i 'error:' "$file" | tee -a "$full_log"
  echo '************************************************' >> "$full_log"
  echo 'Warnings:' | tee -a "$full_log"
  echo '************************************************' >> "$full_log"
  grep -i 'warning:' "$file" | tee -a "$full_log"
  echo '\n\n\n' >> "$full_log"
done

echo 'Saved results to ' "$full_log"
