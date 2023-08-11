# while True:
#     try:
#         num_string = input("Enter numbers separated by comma").split(",")
#         nums = map(int, num_string)
#         break
#     except Exception as e:
#         print(f"Cause exception as {e}")
#     finally:
#         print("nice attempt")

# f = open("test.txt", "w")
# f.write("Hello world!")
# f.close()


# with open("test.txt", "r") as f:
#     file_data = f.read()
    
# print(file_data)


def create_cast_list(filename):
    cast_list = []
    #use with to open the file filename
    #use the for loop syntax to process each line
    #and add the actor name to cast_list
    with open(filename, "r") as f:
        for line in f:
            cast_list.append(line.split(",")[0])

    return cast_list

cast_list = create_cast_list('flying_circus_cast.txt')
for actor in cast_list:
    print(actor)