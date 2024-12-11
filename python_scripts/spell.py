import csv
from spellchecker import SpellChecker

# Initialize the SpellChecker
spell = SpellChecker()

# Function to correct the spelling of a sentence using pyspellchecker
def correct_spelling_with_pyspellcheck(text):
    words = text.split()
    corrected_words = [spell.correction(word) for word in words]
    corrected_text = " ".join(corrected_words)
    return corrected_text

# Function to process the CSV file
def process_csv(input_csv, output_csv):
    with open(input_csv, mode='r', newline='', encoding='utf-8') as infile:
        reader = csv.reader(infile)
        with open(output_csv, mode='w', newline='', encoding='utf-8') as outfile:
            writer = csv.writer(outfile)
            for row in reader:
                corrected_row = []
                for cell in row:
                    # Correct the spelling of each sentence in the CSV
                    corrected_text = correct_spelling_with_pyspellcheck(cell)
                    corrected_row.append(corrected_text)
                writer.writerow(corrected_row)

# Specify the input and output CSV files
input_csv = "input_csv.csv"  # Replace with your actual file path
output_csv = "output_csv.csv"  # Replace with your desired output file path

# Process the CSV file and write the corrected text to the output file
process_csv(input_csv, output_csv)

print("CSV processed and corrected successfully.")

