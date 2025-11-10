import pandas as pd
import os

folder_path = "Milestone 2 Deliverables\S2"  # change this to your folder path
results = []

for file in os.listdir(folder_path):
    if file.lower().endswith(".csv"):
        filepath = os.path.join(folder_path, file)
        df = pd.read_csv(filepath)

        required_cols = ["time_ms", "position_step", "position_deg", "target_step"]
        if not all(col in df.columns for col in required_cols):
            print(f"Skipping {file}: missing required columns") 
            continue

        # find first index where position_step == target_step
        start_idx = df.index[df["position_step"] == df["target_step"]]
        if len(start_idx) == 0:
            print(f"{file}: never reached target_step")
            continue

        start_idx = start_idx[0]
        df_valid = df.iloc[start_idx:]  # only use data after that point

        mean_val = df_valid["position_deg"].mean()
        amplitude = (df_valid["position_deg"].max() - df_valid["position_deg"].min()) / 2

        results.append((file, amplitude))

if results:
    df_summary = pd.DataFrame(results, columns=["file", "amplitude"])
    max_row = df_summary.loc[df_summary["amplitude"].idxmax()]
    min_row = df_summary.loc[df_summary["amplitude"].idxmin()]

    print("\nSummary:")
    print(f"Highest amplitude: {max_row['amplitude']:.3f} from {max_row['file']}")
    print(f"Lowest amplitude: {min_row['amplitude']:.3f} from {min_row['file']}")
else:
    print("No valid data found in any CSVs.")
