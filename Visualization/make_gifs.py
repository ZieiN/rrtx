import glob
from PIL import Image


for imageNum in range(1, 4):
    for id in range(20):
        # filepaths
        fp_in = '../output/ImagesMap' + str(imageNum) + "/test" + str(id) + "/"
        # fp_in = "/path/to/image_*.png"
        fp_out = '../output/GIF_images/Map_' + str(imageNum) + "-test_" + str(id) + "-GIFimage.gif"

        imagesFiles = []
        for i in range(50):
            imagesFiles.append(fp_in+str(i)+".png")
        # https://pillow.readthedocs.io/en/stable/handbook/image-file-formats.html#gif
        img, *imgs = [Image.open(f) for f in imagesFiles]
        print("saving..",imageNum, id)
        img.save(fp=fp_out, format='GIF', append_images=imgs,
                 save_all=True, duration=200, loop=0)