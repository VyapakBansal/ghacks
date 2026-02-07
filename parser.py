def openRawTextFile() -> list[str]:
    with open(r"C:\Users\nateo\OneDrive\Desktop\Coding\ghacks\ghacks\ghacks\Console_2026-02-07_13-44-13.txt", "r", errors = "replace") as file:
        filestring = file.read()
        splitfilelist = filestring.split("#")
        del splitfilelist[0]
        newlist = []
        for text in splitfilelist:
            newlist.append(text[:208])
        return newlist

def convertListToCSV(newlist: list[str]) -> None:
    with open(r"C:\Users\nateo\OneDrive\Desktop\Coding\ghacks\ghacks\ghacks\datacsv.csv", "w") as csv:
        header = "LOG TYPE, PORT,,,,,,,,,Solution Status, Position Type, Latitude, Longitude, Height, Undulation, Datum ID, Latitude std dvn, Longitude std dvn, height std dvn, Station ID, Differential Age, Solution Age, #Satellites, #Satellites Used, #Satellites w/ L1/E1/B1 used, #Satellites w/ multi-frequency"
        # csv.write(header)
        for element in newlist:
            csv.write(element + "\n")

if __name__ == "__main__":
    mylist = openRawTextFile()
    convertListToCSV(mylist)
