using System;
using System.Collections.Generic;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;

namespace TGC.MonoGame.TP;

public class Menu
{
    private readonly string[] _menuItems = ["Iniciar", "Opciones", "Salir"];
    private Texture2D _menuBg;
    private Texture2D _titleImage;
    private SpriteFont _menuFont;
    private int _menuIndex = 0;
    private int _selectedTankIndex = 0;
    private int _playerTankIndex = 0;
    private float _previewAngle = 0f;
    private Matrix _previewView, _previewProj;
    private Vector3 _previewCamTarget = Vector3.Zero;
    private Vector3 _previewCamPos = new Vector3(0f, 1.6f, 4.2f);
    private GraphicsDevice _graphicsDevice;
    private SpriteBatch _spriteBatch;
    private Texture2D _pixel;

    public void Draw(List<TankEntry> tankEntries)
    {
        DrawMenuBackground();
        DrawMenuTankPreview(tankEntries);
        DrawMenu();
    }

    private void DrawMenuTankPreview(List<TankEntry> tankEntries)
    {
        _graphicsDevice.BlendState = BlendState.Opaque;
        _graphicsDevice.DepthStencilState = DepthStencilState.Default;
        _graphicsDevice.RasterizerState = RasterizerState.CullNone;
        _graphicsDevice.SamplerStates[0] = SamplerState.LinearWrap;
        
        if (tankEntries.Count == 0) return;
        var entry = tankEntries[Math.Clamp(_selectedTankIndex, 0, tankEntries.Count - 1)];
        if (entry?.Model == null) return;

        var world =
            Matrix.CreateRotationY(_previewAngle) *
            Matrix.CreateScale(entry.scale) *
            Matrix.CreateTranslation(0f, entry.posY, 0f);

        DrawModel(entry.Model, world, _previewView, _previewProj, entry.Texture, entry.effect);
    }

    private void DrawModel(Model model, Matrix world, Matrix view, Matrix proj, Texture2D texture, Effect effect)
    {
        var boneTransforms = new Matrix[model.Bones.Count];
        model.CopyAbsoluteBoneTransformsTo(boneTransforms);

        foreach (var mesh in model.Meshes)
        {
            var meshWorld = boneTransforms[mesh.ParentBone.Index] * world;
            foreach (var part in mesh.MeshParts)
            {
                var fx = effect.Clone();
                fx.Parameters["World"]?.SetValue(meshWorld);
                fx.Parameters["View"]?.SetValue(view);
                fx.Parameters["Projection"]?.SetValue(proj);
                fx.Parameters["ModelTexture"]?.SetValue(texture);
                part.Effect = fx;
            }

            mesh.Draw();
        }
    }

    private void DrawMenuBackground()
    {
        var vp = _graphicsDevice.Viewport;
        var dst = GetCoverRect(vp.Width, vp.Height, _menuBg.Width, _menuBg.Height);

        // Dibujar la imagen a pantalla completa (cover, puede recortar un poquito)
        _spriteBatch.Begin(SpriteSortMode.Deferred, BlendState.AlphaBlend);
        _spriteBatch.Draw(_menuBg, destinationRectangle: dst, color: Color.White);
        _spriteBatch.End();
    }

    // Calcula un rectángulo "cover" (llena pantalla manteniendo aspecto, con leve recorte)
    private Rectangle GetCoverRect(int viewW, int viewH, int texW, int texH)
    {
        var viewRatio = (float)viewW / viewH;
        var texRatio = (float)texW / texH;

        int w, h, x, y;
        if (texRatio > viewRatio)
        {
            // textura “más ancha”: igualamos alto, recortamos lados
            h = viewH;
            w = (int)(h * texRatio);
            x = (viewW - w) / 2;
            y = 0;
        }
        else
        {
            // textura “más alta”: igualamos ancho, recortamos arriba/abajo
            w = viewW;
            h = (int)(w / texRatio);
            x = 0;
            y = (viewH - h) / 2;
        }

        return new Rectangle(x, y, w, h);
    }

    private void DrawMenu()
    {
        var pantalla = _graphicsDevice.Viewport;
        _spriteBatch.Begin(SpriteSortMode.Deferred, BlendState.AlphaBlend);
        _spriteBatch.Draw(_pixel, new Rectangle(0, 0, pantalla.Width, pantalla.Height), new Color(0, 0, 0, 160));
        
        var targetW = pantalla.Width / 3;
        
        var scale = (float)targetW / _titleImage.Width;
        var drawW = (int)(_titleImage.Width * scale);
        var drawH = (int)(_titleImage.Height * scale);
        
        var x = (pantalla.Width - drawW) / 2;
        var y2 = pantalla.Height * 0.05f;
        
        _spriteBatch.Draw(
            _titleImage,
            destinationRectangle: new Rectangle(x, (int)y2, drawW, drawH),
            color: Color.White
        );

        // Items
        var y = pantalla.Height * 0.6f;
        for (var i = 0; i < _menuItems.Length; i++)
        {
            var text = _menuItems[i];
            var size = _menuFont.MeasureString(text);
            var pos = new Vector2(pantalla.Width / 2f - size.X / 2f, y);

            if (i == _menuIndex)
            {
                // Resaltar selección
                var pad = new Vector2(16, 8);
                var rect = new Rectangle((int)(pos.X - pad.X), (int)(pos.Y - pad.Y),
                    (int)(size.X + pad.X * 2), (int)(size.Y + pad.Y * 2));
                _spriteBatch.Draw(_pixel, rect, new Color(60, 60, 60));
            }

            _spriteBatch.DrawString(_menuFont, text, pos, i == _menuIndex ? Color.White : Color.Silver);
            y += size.Y + 18;
        }

        // Hint
        var hint = "Usa  y Enter"; //↑/↓
        var hintSize = _menuFont.MeasureString(hint);
        _spriteBatch.DrawString(_menuFont, hint, new Vector2(pantalla.Width - hintSize.X - 20, pantalla.Height - hintSize.Y - 20),
            Color.DimGray);

        _spriteBatch.End();
    }

    public void LoadContent(ContentManager content, string contentFolderTextures, GraphicsDevice graphicsDevice,
        string contentFolderSpriteFonts)
    {
        _graphicsDevice = graphicsDevice;
        _menuFont = content.Load<SpriteFont>(contentFolderSpriteFonts + "CascadiaCode/CascadiaCodePL");
        _spriteBatch = new SpriteBatch(graphicsDevice);
        _pixel = new Texture2D(graphicsDevice, 1, 1);
        _pixel.SetData(new[] { Color.White });
        _previewView = Matrix.CreateLookAt(new Vector3(0f, 1.6f, 4.2f), Vector3.Zero, Vector3.Up);
        _previewProj = Matrix.CreatePerspectiveFieldOfView(
            MathHelper.PiOver4, graphicsDevice.Viewport.AspectRatio, 0.1f, 1000f);

        _menuBg = content.Load<Texture2D>(contentFolderTextures + "menu_bg");
        _titleImage = content.Load<Texture2D>(contentFolderTextures + "chaskibum");
    }

    public void Update(KeyboardState keyboardState, KeyboardState kbPrev, GameTime gameTime, TGCGame tgcGame,
        List<TankEntry> tankEntries)
    {
        _previewAngle += (float)gameTime.ElapsedGameTime.TotalSeconds * 0.6f;
        
        if (keyboardState.IsKeyDown(Keys.Up) && !kbPrev.IsKeyDown(Keys.Up))
            _menuIndex = (_menuIndex - 1 + _menuItems.Length) % _menuItems.Length;

        if (keyboardState.IsKeyDown(Keys.Down) && !kbPrev.IsKeyDown(Keys.Down))
            _menuIndex = (_menuIndex + 1) % _menuItems.Length;

        if (keyboardState.IsKeyDown(Keys.Left) && !kbPrev.IsKeyDown(Keys.Left))
            _selectedTankIndex = (_selectedTankIndex - 1 + tankEntries.Count) % tankEntries.Count;

        if (keyboardState.IsKeyDown(Keys.Right) && !kbPrev.IsKeyDown(Keys.Right))
            _selectedTankIndex = (_selectedTankIndex + 1) % tankEntries.Count;

        if (keyboardState.IsKeyDown(Keys.Enter) && !kbPrev.IsKeyDown(Keys.Enter))
        {
            var choice = _menuItems[_menuIndex];
            if (choice.StartsWith("Iniciar"))
            {
                _playerTankIndex = _selectedTankIndex;
                tgcGame.StartGame();
            }
            else if (choice.StartsWith("Opciones"))
            {
                // TODO: abrir submenú de opciones (volúmenes, dificultad, etc.)                                       
            }
            else if (choice.StartsWith("Salir"))
            {
                tgcGame.Exit();
            }
        }

        // Permite salir también con Escape
        if (keyboardState.IsKeyDown(Keys.Escape) && !kbPrev.IsKeyDown(Keys.Escape))
            tgcGame.Exit();
    }
}