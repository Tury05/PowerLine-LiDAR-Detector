def main():
    f = open("outputfile.txt", "r")
    out = open("2D.txt", "w")
    
    out.write("X,Y,Z\n")

    for line in f.readlines()[1:]:
        content = line.split(",")
        if content[3] == "-1.000\n":
            out.write(",".join(content))
    
    f.close()
    out.close()

if __name__ == '__main__':
    main()