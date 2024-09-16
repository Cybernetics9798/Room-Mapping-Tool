import numpy as np
import open3d as o3d
import serial


if __name__ == "__main__":
    '''
    #Connect the port to the MCU
    s = serial.Serial('COM3', 115200, timeout=10)
    print("Opening: " + s.name)
    # reset the buffers of the UART port to delete the remaining data in the buffers
    s.reset_output_buffer()
    s.reset_input_buffer()
    # wait for user's signal to start the program
    input("Press Enter to start communication...")
    # send the character 's' to MCU via UART, which will signal MCU to start the transmission
    # the encode() and decode() function are needed to convert string to bytes
    # because pyserial library functions work with type "bytes"
    s.write('s'.encode())

    #create a new file for writing
    f = open("tof_radar.xyz", "w")
    #slices = int(s.readline().decode())
    '''
    slices = 11
    print(slices)
    '''
    # recieve measurements
    for i in range(slices):
        for j in range(8):
            data = s.readline()
            print(data.decode())
            coord = data.decode().split(",")
            y = float(coord[0])
            z = float(coord[1])
            x = float(coord[2])
            f.write('{:4.8} {:4.8} {:4.8}\n'.format(x, y, z))

    # close the port
    print("Closing: " + s.name)
    s.close()
    f.close()
    '''
    #Read the test data in from the file we created        
    print("Read in the prism point cloud data (pcd)")
    pcd = o3d.io.read_point_cloud("tof_radar.xyz", format="xyz")

    #Lets see what our point cloud data looks like numerically       
    print("The PCD array:")
    print(np.asarray(pcd.points))

    #   For creating a lineset we will need to tell the packahe which vertices need connected
    #   Give each vertex a unique number
    yz_slice_vertex = []
    samplePoints = 8
    for i in range(slices*samplePoints):
        yz_slice_vertex.append([i])
    
    #   Define coordinates to connect lines in each yz slice
    lines = []
    '''
    for i in range(0, samplePoints*slices, samplePoints):
        for j in range(0, samplePoints):
            lines.append([yz_slice_vertex[i + j], yz_slice_vertex[(i + j + 1)%samplePoints]])
        #lines.append([yz_slice_vertex[samplePoints - 1], yz_slice_vertex[0]])
    '''
    for x in range(0, samplePoints*slices, 8):
        lines.append([yz_slice_vertex[x], yz_slice_vertex[x + 1]])
        lines.append([yz_slice_vertex[x + 1], yz_slice_vertex[x + 2]])
        lines.append([yz_slice_vertex[x + 2], yz_slice_vertex[x + 3]])
        lines.append([yz_slice_vertex[x + 3], yz_slice_vertex[x + 4]])
        lines.append([yz_slice_vertex[x + 4], yz_slice_vertex[x + 5]])
        lines.append([yz_slice_vertex[x + 5], yz_slice_vertex[x + 6]])
        lines.append([yz_slice_vertex[x + 6], yz_slice_vertex[x + 7]])
        lines.append([yz_slice_vertex[x + 7], yz_slice_vertex[x]])
    
    #Define coordinates to connect lines between current and next yz slice
    '''
    for i in range(0, slices*8, 8):
        for j in range(samplePoints - 1):
            lines.append([yz_slice_vertex[i+j], yz_slice_vertex[i + j + 7]])
    '''
    
    for i in range(0, samplePoints*(slices - 1) - 1, 8):
        lines.append([yz_slice_vertex[i], yz_slice_vertex[i + 8]])
        lines.append([yz_slice_vertex[i + 1], yz_slice_vertex[i + 9]])
        lines.append([yz_slice_vertex[i + 2], yz_slice_vertex[i + 10]])
        lines.append([yz_slice_vertex[i + 3], yz_slice_vertex[i + 11]])
        lines.append([yz_slice_vertex[i + 4], yz_slice_vertex[i + 12]])
        lines.append([yz_slice_vertex[i + 5], yz_slice_vertex[i + 13]])
        lines.append([yz_slice_vertex[i + 6], yz_slice_vertex[i + 14]])
        lines.append([yz_slice_vertex[i + 7], yz_slice_vertex[i + 15]])
    
    
    #This line maps the lines to the 3d coordinate vertices
    line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),lines=o3d.utility.Vector2iVector(lines))

    #Lets see what our point cloud data with lines looks like graphically       
    o3d.visualization.draw_geometries([line_set])
