import pyglet

class GameObject:
    def __init__(self, posx, posy, image=None):
        self.posx = posx
        self.posy = posy
        if image is not None:
            image = pyglet.image.load(image)
            self.width = image.width
            self.height = image.height
            self.sprite = pyglet.sprite.Sprite(image, x=self.posx, y=self.posy)


    def draw(self):
        self.sprite.draw()

    def set_pos(self, x, y):
        self.sprite.x = x
        self.sprite.y = y

    def get_width(self):
        return self.width

    def get_height(self):
        return self.height
