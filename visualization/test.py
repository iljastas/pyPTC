


from PIL import Image, ImageFont, ImageDraw

text = 'TEST'
font = Image.open("visualization/tractor.jpg")
width, height = font.getsize()

image1 = Image.new('RGBA', (200, 150), (0, 128, 0, 92))
draw1 = ImageDraw.Draw(image1)
# draw1.text((0, 0), text=text, font=font, fill=(255, 128, 0))

image2 = Image.new('RGBA', (width, height), (0, 0, 128, 92))
draw2 = ImageDraw.Draw(image2)
draw2.text((0, 0), text=text, font=font, fill=(0, 255, 128))

image2 = image2.rotate(30, expand=1)

px, py = 10, 10
sx, sy = image2.size
image1.paste(image2, (px, py, px + sx, py + sy), image2)

image1.show()

