using System;
using System.Collections.Generic;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Audio;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using Microsoft.Xna.Framework.Media;

namespace TGC.MonoGame.TP;

public class Menu
{
    private readonly string[] _menuItems = ["Iniciar", "Opciones", "Salir"];
    
    private enum MenuState { Main, Options }
    private MenuState _state = MenuState.Main;
    
    private readonly int[] _matchMinutesOptions = { 3, 5, 10, 15 };
    private int _matchMinutesIndex = 1; // default: 5 min

    private int _enemyCount = 5; // default
    private const int EnemyMin = 1;
    private const int EnemyMax = 20;
    
    private readonly string[] _optionsItems = ["Tiempo de partida", "Tanques enemigos", "Volver"];
    private int _optionsIndex = 0;

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

    // Audio
    private Song _menuMusic;
    private SoundEffect _selectSound;
    private SoundEffect _moveSound;
    private bool _musicStarted = false;

    private TimeSpan SelectedMatchTime => TimeSpan.FromMinutes(_matchMinutesOptions[_matchMinutesIndex]);
    private int SelectedEnemyCount => _enemyCount;
    private int SelectedPlayerTankIndex => _playerTankIndex;

    public void Draw(List<TankEntry> tankEntries, Texture2D treadmillTexture)
    {
        // Iniciar música de fondo si no está sonando
        if (!_musicStarted && _menuMusic != null)
        {
            try
            {
                MediaPlayer.IsRepeating = true;
                MediaPlayer.Volume = 0.3f;
                MediaPlayer.Play(_menuMusic);
                _musicStarted = true;
            }
            catch
            {
                // Si hay error al reproducir, continuar sin música
            }
        }

        DrawMenuBackground();
        
        if (_state == MenuState.Main)
            DrawMenuTankPreview(tankEntries, treadmillTexture);
        
        if (_state == MenuState.Main)
            DrawMainMenu();
        else
            DrawOptionsMenu();
    }

    private void DrawMenuTankPreview(List<TankEntry> tankEntries, Texture2D treadmillTexture)
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

        DrawModel(entry.Model, world, _previewView, _previewProj, entry.Texture, entry.effect, treadmillTexture);
    }

    private void DrawModel(Model model, Matrix world, Matrix view, Matrix proj, Texture2D texture, Effect effect, Texture2D treadmillTexture)
    {
        var boneTransforms = new Matrix[model.Bones.Count];
        model.CopyAbsoluteBoneTransformsTo(boneTransforms);
        var fx = effect.Clone();
        
        foreach (var mesh in model.Meshes)
        {
            var meshWorld = boneTransforms[mesh.ParentBone.Index] * world;
            
            foreach (var part in mesh.MeshParts)
                part.Effect = fx;
            
            fx.Parameters["World"]?.SetValue(meshWorld);
            fx.Parameters["WorldViewProjection"].SetValue(meshWorld * view * proj);
            fx.Parameters["ModelTexture"]?.SetValue(texture);
            if (mesh.Name.Contains("Treadmill"))
            {
                fx.Parameters["ModelTexture"]?.SetValue(treadmillTexture);
            }
            
            mesh.Draw();
        }
    }

    private void DrawMenuBackground()
    {
        var vp = _graphicsDevice.Viewport;
        var dst = GetCoverRect(vp.Width, vp.Height, _menuBg.Width, _menuBg.Height);

        _spriteBatch.Begin(SpriteSortMode.Deferred, BlendState.AlphaBlend);
        _spriteBatch.Draw(_menuBg, destinationRectangle: dst, color: Color.White);
        _spriteBatch.End();
    }

    private Rectangle GetCoverRect(int viewW, int viewH, int texW, int texH)
    {
        var viewRatio = (float)viewW / viewH;
        var texRatio = (float)texW / texH;

        int w, h, x, y;
        if (texRatio > viewRatio)
        {
            h = viewH;
            w = (int)(h * texRatio);
            x = (viewW - w) / 2;
            y = 0;
        }
        else
        {
            w = viewW;
            h = (int)(w / texRatio);
            x = 0;
            y = (viewH - h) / 2;
        }

        return new Rectangle(x, y, w, h);
    }
    
    private void DrawMainMenu()
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

        _spriteBatch.Draw(_titleImage, destinationRectangle: new Rectangle(x, (int)y2, drawW, drawH), color: Color.White);

        // Items
        var y = pantalla.Height * 0.6f;
        for (var i = 0; i < _menuItems.Length; i++)
        {
            var text = _menuItems[i];
            var size = _menuFont.MeasureString(text);
            var pos = new Vector2(pantalla.Width / 2f - size.X / 2f, y);

            if (i == _menuIndex)
            {
                var pad = new Vector2(16, 8);
                var rect = new Rectangle((int)(pos.X - pad.X), (int)(pos.Y - pad.Y),
                    (int)(size.X + pad.X * 2), (int)(size.Y + pad.Y * 2));
                _spriteBatch.Draw(_pixel, rect, new Color(60, 60, 60));
            }

            _spriteBatch.DrawString(_menuFont, text, pos, i == _menuIndex ? Color.White : Color.Silver);
            y += size.Y + 18;
        }

        // ayuda
        var hint = "Usa ↑/↓ para moverte en el menú, ←/→ para cambiar el tanque y Enter para seleccionar";
        var hintSize = _menuFont.MeasureString(hint);
        _spriteBatch.DrawString(_menuFont, hint,
            new Vector2(pantalla.Width - hintSize.X - 20, pantalla.Height - hintSize.Y - 20),
            Color.DimGray);
        
        _spriteBatch.End();
    }

    private void DrawOptionsMenu()
    {
        var pantalla = _graphicsDevice.Viewport;
        _spriteBatch.Begin(SpriteSortMode.Deferred, BlendState.AlphaBlend);

        // panel
        var panelW = (int)(pantalla.Width * 0.6f);
        var panelH = (int)(pantalla.Height * 0.5f);
        var panelX = (pantalla.Width - panelW) / 2;
        var panelY = (pantalla.Height - panelH) / 2;

        _spriteBatch.Draw(_pixel, new Rectangle(panelX, panelY, panelW, panelH), new Color(0, 0, 0, 180));

        var title = "Opciones";
        var ts = _menuFont.MeasureString(title);
        _spriteBatch.DrawString(_menuFont, title, new Vector2(panelX + (panelW - ts.X) / 2, panelY + 18), Color.White);

        var y = panelY + 80;
        DrawOptionRow($"Tiempo de partida: {_matchMinutesOptions[_matchMinutesIndex]} min   (←/→)", 0, y, panelX, panelW);
        y += (int)_menuFont.LineSpacing + 20;

        DrawOptionRow($"Tanques enemigos: {_enemyCount}   (←/→)", 1, y, panelX, panelW);
        y += (int)_menuFont.LineSpacing + 20;

        DrawOptionRow("Volver", 2, y, panelX, panelW);

        _spriteBatch.End();
    }

    private void DrawOptionRow(string text, int rowIndex, int y, int panelX, int panelW)
    {
        var size = _menuFont.MeasureString(text);
        var x = panelX + (panelW - size.X) / 2;
        
        if (_optionsIndex == rowIndex)
        {
            var pad = new Vector2(16, 8);
            var rect = new Rectangle((int)(x - pad.X), (int)(y - pad.Y),
                (int)(size.X + pad.X * 2), (int)(size.Y + pad.Y * 2));
            _spriteBatch.Draw(_pixel, rect, new Color(60, 60, 60));
        }

        _spriteBatch.DrawString(_menuFont, text, new Vector2(x, y),
            _optionsIndex == rowIndex ? Color.White : Color.Silver);
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

        // Cargar audio (opcional - no falla si no existen los archivos)
        try
        {
            _menuMusic = content.Load<Song>("Music/menu_music");
        }
        catch
        {
            // Música no encontrada, continuar sin ella
        }

        try
        {
            _selectSound = content.Load<SoundEffect>("Sounds/menu_select");
        }
        catch
        {
            // Sonido no encontrado
        }

        try
        {
            _moveSound = content.Load<SoundEffect>("Sounds/menu_move");
        }
        catch
        {
            // Sonido no encontrado
        }
    }

    public void Update(KeyboardState keyboardState, KeyboardState kbPrev, GameTime gameTime, TGCGame tgcGame,
        List<TankEntry> tankEntries)
    {
        _previewAngle += (float)gameTime.ElapsedGameTime.TotalSeconds * 0.6f;

        if (_state == MenuState.Main)
        {
            // Navegación menú principal
            if (keyboardState.IsKeyDown(Keys.Up) && !kbPrev.IsKeyDown(Keys.Up))
            {
                _menuIndex = (_menuIndex - 1 + _menuItems.Length) % _menuItems.Length;
                PlaySound(_moveSound);
            }

            if (keyboardState.IsKeyDown(Keys.Down) && !kbPrev.IsKeyDown(Keys.Down))
            {
                _menuIndex = (_menuIndex + 1) % _menuItems.Length;
                PlaySound(_moveSound);
            }

            if (tankEntries.Count > 0)
            {
                if (keyboardState.IsKeyDown(Keys.Left) && !kbPrev.IsKeyDown(Keys.Left))
                {
                    _selectedTankIndex = (_selectedTankIndex - 1 + tankEntries.Count) % tankEntries.Count;
                    PlaySound(_moveSound);
                }

                if (keyboardState.IsKeyDown(Keys.Right) && !kbPrev.IsKeyDown(Keys.Right))
                {
                    _selectedTankIndex = (_selectedTankIndex + 1) % tankEntries.Count;
                    PlaySound(_moveSound);
                }
            }

            if (keyboardState.IsKeyDown(Keys.Enter) && !kbPrev.IsKeyDown(Keys.Enter))
            {
                PlaySound(_selectSound);
                var choice = _menuItems[_menuIndex];
                if (choice.StartsWith("Iniciar"))
                {
                    _playerTankIndex = _selectedTankIndex;
                    StopMenuMusic();
                    tgcGame.StartGame(SelectedMatchTime, SelectedEnemyCount, SelectedPlayerTankIndex);
                }
                else if (choice.StartsWith("Opciones"))
                {
                    _state = MenuState.Options;
                    _optionsIndex = 0;
                }
                else if (choice.StartsWith("Salir"))
                {
                    tgcGame.Exit();
                }
            }

            if (keyboardState.IsKeyDown(Keys.Escape) && !kbPrev.IsKeyDown(Keys.Escape))
                tgcGame.Exit();
        }
        else // MenuState.Options
        {
            if (keyboardState.IsKeyDown(Keys.Up) && !kbPrev.IsKeyDown(Keys.Up))
            {
                _optionsIndex = (_optionsIndex - 1 + _optionsItems.Length) % _optionsItems.Length;
                PlaySound(_moveSound);
            }

            if (keyboardState.IsKeyDown(Keys.Down) && !kbPrev.IsKeyDown(Keys.Down))
            {
                _optionsIndex = (_optionsIndex + 1) % _optionsItems.Length;
                PlaySound(_moveSound);
            }
            
            if (keyboardState.IsKeyDown(Keys.Left) && !kbPrev.IsKeyDown(Keys.Left))
            {
                if (_optionsIndex == 0) // tiempo
                {
                    _matchMinutesIndex = (_matchMinutesIndex - 1 + _matchMinutesOptions.Length) % _matchMinutesOptions.Length;
                    PlaySound(_moveSound);
                }
                else if (_optionsIndex == 1) // enemigos
                {
                    _enemyCount = Math.Max(EnemyMin, _enemyCount - 1);
                    PlaySound(_moveSound);
                }
            }

            if (keyboardState.IsKeyDown(Keys.Right) && !kbPrev.IsKeyDown(Keys.Right))
            {
                if (_optionsIndex == 0) // tiempo
                {
                    _matchMinutesIndex = (_matchMinutesIndex + 1) % _matchMinutesOptions.Length;
                    PlaySound(_moveSound);
                }
                else if (_optionsIndex == 1) // enemigos
                {
                    _enemyCount = Math.Min(EnemyMax, _enemyCount + 1);
                    PlaySound(_moveSound);
                }
            }
            
            if (keyboardState.IsKeyDown(Keys.Enter) && !kbPrev.IsKeyDown(Keys.Enter))
            {
                PlaySound(_selectSound);
                _state = MenuState.Main;
            }
            
            if (keyboardState.IsKeyDown(Keys.Escape) && !kbPrev.IsKeyDown(Keys.Escape))
            {
                PlaySound(_selectSound);
                _state = MenuState.Main;
            }
        }
    }

    // Métodos auxiliares para audio
    private void PlaySound(SoundEffect sound)
    {
        try
        {
            sound?.Play(volume: 0.5f, pitch: 0f, pan: 0f);
        }
        catch
        {
            // Ignorar errores de reproducción
        }
    }

    public void StopMenuMusic()
    {
        try
        {
            if (MediaPlayer.State == MediaState.Playing)
            {
                MediaPlayer.Stop();
            }
            _musicStarted = false;
        }
        catch
        {
            // Ignorar errores
        }
    }
}
