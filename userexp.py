import csv

def count_correct_responses(csv_file_path):
    correct_count = 0
    total_count = 0

    with open(csv_file_path, 'r') as file:
        reader = csv.reader(file)

        # If your CSV has a header, uncomment the next line
        # next(reader)

        for row in reader:
            try:
                correct_data = row[9]   # Column 10 (index starts from 0)
                user_data = row[10]     # Column 11

                if correct_data == user_data:
                    correct_count += 1

                total_count += 1

            except IndexError:
                print("Skipping row due to missing columns:", row)

    return correct_count, total_count


# ---- Usage ----
file_path = "E:\MTP\himanshuuu_2026-03-25_18-03-03.csv"  # Replace with your CSV file path

correct, total = count_correct_responses(file_path)

print(f"Total Experiments: {total}")
print(f"Correct Responses: {correct}")
print(f"Accuracy: {(correct/total)*100:.2f}%")