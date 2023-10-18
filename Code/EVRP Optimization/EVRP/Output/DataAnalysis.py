with open("RawOutput.txt", "r") as file:
    for line in file:
        raw_row = [x for x in line.strip("\n").split(",")]
        print(raw_row)
        
