import pandas
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
import os

# Recording file list
recording_files = [
    "144821",
    "145043",
    "145233",
    "145641",
    "150001"
]

# Function to calculate accuracy between original and predicted steering
def accuracy_calculation(original, predicted, range=0.09):
    """
    Calculate the accuracy between original and predicted steering values.
    This function computes the percentage of valid predictions.
    """
    # Reset index to avoid misalignment
    original = original.reset_index(drop=True)
    predicted = predicted.reset_index(drop=True)

    # Only consider non-zero ground steering values 
    is_nonzero = original.abs() != 0
    original_filtered = original[is_nonzero]
    predicted_filtered = predicted[is_nonzero]
    
    # Compare predictions within acceptable range
    correct_frames = abs(predicted_filtered - original_filtered) <= range
    total_matched = correct_frames.sum()
    total_frames = len(original_filtered)
    
    accuracy = (total_matched / total_frames) * 100  # Convert to percentage
    return accuracy

# Path to the CSV file
csv_dir = "logs/csv"
prev_csv_dir = "logs/csv-prev"  # Directory where previous commit CSVs are stored
output_dir = "logs/performance-plots"
os.makedirs(output_dir, exist_ok=True)  # Ensure the output directory exists

for name in recording_files: # Loop through each recording file
    csv_file = os.path.join(csv_dir, f"{name}.csv")
    prev_csv_file = os.path.join(prev_csv_dir, f"{name}.csv")
    output_path = os.path.join(output_dir, f"{name}.png")  # .png file for each recording

    # Read the CSV file
    data = pandas.read_csv(csv_file)

    # Remove duplicated timestamps, if any
    data = data[~data["timestamp_us"].duplicated()]
    
    # Check if the required columns are present
    required_columns = ["timestamp_us", "predicted_steering", "original_steering"]
    missing_columns = [col for col in required_columns if col not in data.columns]
    if missing_columns:
        print(f" Error: Missing columns {missing_columns} in CSV file.")
        continue

    # Check if previous commit CSV exists for this recording
    previous_commit_exists = os.path.exists(prev_csv_file)

    # Calculate accuracy for current commit
    current_accuracy = accuracy_calculation(data["original_steering"], data["predicted_steering"])
    print(f"{name}: Current accuracy: {current_accuracy:.2f}%")

    # Calculate accuracy for previous commit if available
    if previous_commit_exists:
            previous_data = pandas.read_csv(prev_csv_file)

            min_len = min(len(previous_data), len(data))
            prev_predicted = previous_data["predicted_steering"].iloc[:min_len]
            orig_steering = data["original_steering"].iloc[:min_len]
            previous_accuracy = accuracy_calculation(orig_steering, prev_predicted)
            accuracy_variation = abs(current_accuracy - previous_accuracy)

            print(f"{name}: Previous accuracy: {previous_accuracy:.2f}%")
            print(f"{name}: Accuracy variation: {accuracy_variation:.2f}%")
            
    else:
        previous_accuracy = None

    # Extract data for plotting
    timestamp_us = data["timestamp_us"] - data["timestamp_us"].min()  # x-axis
    predicted_steering = data["predicted_steering"]  # y-axis (first line)
    original_steering = data["original_steering"]  # y-axis (second line)
        
    
    # Plot the data 
    plt.figure(figsize=(10, 6))
    plt.plot(timestamp_us, predicted_steering, label=f"Predicted Steering (Accuracy: {current_accuracy:.1f}%)", color="blue")
    plt.plot(timestamp_us, original_steering, label="Original Steering", color="red")

    if previous_commit_exists:
        plt.plot(previous_data["timestamp_us"], previous_data["predicted_steering"],label=f"Previous Prediction (Accuracy: {previous_accuracy:.1f}%)",color="green", linestyle="--")
    
    # Customize the plot
    plt.xlabel("Timestamp (microseconds)")
    plt.ylabel("GroundSteering Values")
    plt.title(f"GroundSteering Plot - Recording {name}")
    plt.legend()

    # Adjust x-axis to display full microseconds
    plt.ticklabel_format(style='plain', axis='x')

    # Add group watermark
    plt.text(
        0.02, 0.02, "Group: 12", 
        fontsize=12, color='red', alpha=0.4, transform=plt.gca().transAxes
    )

    # Customize the grid to make it smaller
    plt.grid(True, linestyle='--', linewidth=0.4, alpha=0.5)
    plt.tight_layout()

    # Save to file
    plt.savefig(output_path)
    plt.close()  # Close the plot to free memory