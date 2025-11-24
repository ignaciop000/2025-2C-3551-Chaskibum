using System;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;
using TGC.MonoGame.TP.Proyectiles;

namespace TGC.MonoGame.TP.UI;

public class HUD(GraphicsDevice graphicsDevice)
{
    private readonly SpriteBatch _spriteBatch = new(graphicsDevice);
    private SpriteFont _menuFont;
    private SpriteFont _tituloFont;
    private Texture2D _pixel;
    private Texture2D _clockFrame;
    private Texture2D _infoFrame;
    private Texture2D _proyectil1;
    private Texture2D _proyectil2;

    public void LoadContent(ContentManager content, string contentFolderTextures, GraphicsDevice graphicsDevice,
        ICloneable contentFolderSpriteFonts)
    {
        _menuFont = content.Load<SpriteFont>(contentFolderSpriteFonts + "CascadiaCode/CascadiaCodePL");
        _tituloFont = content.Load<SpriteFont>(contentFolderSpriteFonts + "CascadiaCode/CascadiaMonoPL");
        _clockFrame = content.Load<Texture2D>(System.IO.Path.Combine(contentFolderTextures, "fondo_reloj"));
        _infoFrame = content.Load<Texture2D>(System.IO.Path.Combine(contentFolderTextures, "fondo_info"));
        _proyectil1 = content.Load<Texture2D>(System.IO.Path.Combine(contentFolderTextures, "proyectil1"));
        _proyectil2 = content.Load<Texture2D>(System.IO.Path.Combine(contentFolderTextures, "proyectil2"));
        
        // Pixel 1x1 para poder dibujar backgrounds/selecciones                                                          
        _pixel = new Texture2D(graphicsDevice, 1, 1);
        _pixel.SetData([Color.White]);
    }

    private void DrawInfo(float fireCooldown, float fireCooldownMax, ProjectileType currentProjectile, float playerHealth, float playerMaxHealth, int enemyCount)
    {
        var viewport = graphicsDevice.Viewport;
        var text = "Enemigos restantes: " + enemyCount;
        var margin = 20f;
        var pos = new Vector2(margin, margin);
        _spriteBatch.DrawString(_menuFont, text, pos, Color.White);
        // Layout base abajo-izquierda
        
        var baseX = margin;
        
        const float frameInfoScale = 0.4f; 
        var anchoFrameInfo = (int)(_infoFrame.Width * frameInfoScale);
        var largoFrameInfo = (int)(_infoFrame.Height * frameInfoScale);
        var baseY = viewport.Height - margin - largoFrameInfo;
        
        _spriteBatch.Draw(
            _infoFrame,
            destinationRectangle: new Rectangle((int)baseX, (int)baseY, anchoFrameInfo, largoFrameInfo),
            color: Color.White
        );
        var anchoLogo = (anchoFrameInfo - 150) / 2;
        var midPointX = (int) baseX + anchoFrameInfo / 2 - (anchoLogo /2 );
        var midPointY = baseY + anchoLogo / 2;
        DrawProyectil(currentProjectile, midPointX, (int)midPointY, anchoLogo, anchoLogo);
        var readyFrac = 1f - MathHelper.Clamp(fireCooldown / fireCooldownMax, 0f, 1f);
        
        var cdWidth = _infoFrame.Width * 0.26f;
        var cdHeight = 18f;
        
        var cdRect = new Rectangle((int) (baseX + anchoFrameInfo / 2 - cdWidth / 2), (int)(baseY)+ anchoLogo + (anchoLogo / 2) + 30, (int)cdWidth, (int)cdHeight);
        DrawBar(cdRect, 0.35f, 0.35f, 0.35f, 0.6f);
        var fillRect = new Rectangle(cdRect.X, cdRect.Y, (int)(cdRect.Width * readyFrac), cdRect.Height);
        DrawBar(fillRect, 0.2f, 0.8f, 0.2f, 0.9f);
        string cdText = readyFrac >= 0.999f
            ? "Disparo listo"
            : $"Recargando... {(fireCooldownMax - fireCooldown):0.00}s";
        _spriteBatch.DrawString(_menuFont, cdText, new Vector2(cdRect.X, cdRect.Y - 30), Color.White);


        var lifeWidth = _infoFrame.Width * 0.26f;
        var lifeHeight = 22f;
        var lifeRect = new Rectangle((int) (baseX + anchoFrameInfo / 2 - cdWidth / 2), (int)(baseY)+ anchoLogo + (anchoLogo / 2) + 30 + 58, (int)lifeWidth, (int)lifeHeight);
        DrawBar(lifeRect, 0.35f, 0.35f, 0.35f, 0.6f);
        var lifeFrac = MathHelper.Clamp(playerHealth / MathF.Max(1f, playerMaxHealth), 0f, 1f);
        var lifeFill = new Rectangle(lifeRect.X, lifeRect.Y, (int)(lifeRect.Width * lifeFrac), lifeRect.Height);
        _spriteBatch.Draw(_pixel, lifeFill, new Color(1f - lifeFrac, lifeFrac, 0f, 0.95f));

        var lifeText = $"Vida: {(int)playerHealth}/{(int)playerMaxHealth}";
        var lifeTextPos = new Vector2(lifeRect.X, lifeRect.Y - 30);
        _spriteBatch.DrawString(_menuFont, lifeText, lifeTextPos, Color.White);
    }

    private void DrawProyectil(ProjectileType currentProjectile, int x, int y, int ancho, int largo)
    {
        _spriteBatch.Draw(
            currentProjectile == ProjectileTypes.Heavy ? _proyectil1 : _proyectil2,
            destinationRectangle: new Rectangle(x, y, ancho, largo),
            color: Color.White
        );
    }

    /// <summary>
    /// Dibuja la hora del sistema con los sprites numéricos, centrada arriba.
    /// </summary>
    private void DrawClockSpritesTopCenter(float matchTimeSeconds)
    {
        var viewport = graphicsDevice.Viewport;
        
        // Posición del marco: centrado arriba
        var frameScale = 0.2f; 
        var anchoFrameReloj = (int)(_clockFrame.Width * frameScale);
        var largoFrameReloj = (int)(_clockFrame.Height * frameScale);
        var frameRelojX = (int)(viewport.Width * 0.5f - anchoFrameReloj * 0.5f);
        var frameRelojY = 16f; 

        // Dibujamos el marco
        _spriteBatch.Draw(
            _clockFrame,
            destinationRectangle: new Rectangle(frameRelojX, (int)frameRelojY, anchoFrameReloj, largoFrameReloj),
            color: Color.White
        );
        
        var timeStr = FormatTime(matchTimeSeconds);
        var timeSize = _menuFont.MeasureString(timeStr);
        var timePos = new Vector2(viewport.Width / 2f - timeSize.X / 2f, frameRelojY + largoFrameReloj/2 - 12 );
        // Sombra suave
        _spriteBatch.DrawString(_menuFont, timeStr, timePos + new Vector2(1, 1), Color.Black * 0.6f);
        _spriteBatch.DrawString(_menuFont, timeStr, timePos, Color.White);
    }


    public void Draw(float matchTimeSeconds, float fireCooldown, float fireCooldownMax,
        ProjectileType currentProjectile, float playerHealth, float playerMaxHealth, int enemyCount, GameTime gameTime)
    {
        DrawInfo(fireCooldown, fireCooldownMax, currentProjectile, playerHealth, playerMaxHealth, enemyCount);
        DrawClockSpritesTopCenter(matchTimeSeconds);
    }

    public void Begin()
    {
        _spriteBatch.Begin(SpriteSortMode.Deferred, BlendState.AlphaBlend);
    }

    public void End()
    {
        _spriteBatch.End();
    }

    public void DrawHealthBar(Vector2 position, float healthPercentage, int width, int height)
    {
        var backgroundColor = new Color(0.8f, 0.1f, 0.1f, 0.8f);
        var foregroundColor = Color.Green;

        var backgroundRect = new Rectangle((int)position.X - width / 2, (int)position.Y, width, height);
        _spriteBatch.Draw(_pixel, backgroundRect, backgroundColor);

        var foregroundWidth = (int)(width * healthPercentage);
        var foregroundRect = new Rectangle((int)position.X - width / 2, (int)position.Y, foregroundWidth, height);
        _spriteBatch.Draw(_pixel, foregroundRect, foregroundColor);
    }

    private void DrawBar(Rectangle rect, float r, float g, float b, float a)
    {
        // Fondo
        _spriteBatch.Draw(_pixel, rect, new Color(r, g, b, a));
        // Borde sutil
        var border = new Rectangle(rect.X - 1, rect.Y - 1, rect.Width + 2, rect.Height + 2);
        _spriteBatch.Draw(_pixel, border, Color.Black * 0.25f);
    }

    private string FormatTime(float seconds)
    {
        if (seconds < 0f) seconds = 0f;
        int mm = (int)(seconds / 60f);
        int ss = (int)(seconds % 60f);
        return $"{mm:00}:{ss:00}";
    }
    
    public void DrawMensaje(string mensaje, Color color, float time)
    {
        var vp = graphicsDevice.Viewport;

        float scale = 1.5f;
        var font = _tituloFont; 

        var medida = font.MeasureString(mensaje) * scale;
        var centroPantalla = new Vector2(vp.Width / 2f, vp.Height / 2f);        
        var posicion = centroPantalla - medida / 2f;
        
        var alpha = 0f;        
        if(time > 0)
             alpha = 1 - 1f * (time / 5);
        _spriteBatch.Draw(_pixel, new Rectangle(0, 0, vp.Width, vp.Height), new Color(0, 0, 0, alpha) );
        
        _spriteBatch.DrawString(
            font,
            mensaje,
            posicion,
            color,
            0f,
            Vector2.Zero,  
            scale,
            SpriteEffects.None,
            0f
        );
    }
    
    /// <summary>
    /// Dibuja panel de ayuda con los controles del juego
    /// </summary>
    public void DrawHelp()
    {
        var viewport = graphicsDevice.Viewport;
        
        // Panel semi-transparente
        var panelW = 600;
        var panelH = 500;
        var panelX = (viewport.Width - panelW) / 2;
        var panelY = (viewport.Height - panelH) / 2;
        
        _spriteBatch.Draw(_pixel, new Rectangle(panelX, panelY, panelW, panelH), new Color(0, 0, 0, 200));
        
        // Título
        var title = "CONTROLES DEL JUEGO";
        var titleSize = _menuFont.MeasureString(title);
        var titlePos = new Vector2(panelX + (panelW - titleSize.X) / 2, panelY + 20);
        _spriteBatch.DrawString(_menuFont, title, titlePos, Color.Yellow);
        
        // Lista de controles
        var y = panelY + 70;
        var lineHeight = 30;
        var leftMargin = panelX + 40;
        
        var controls = new[]
        {
            "MOVIMIENTO:",
            "  W/A/S/D - Mover tanque",
            "  Shift - Turbo",
            "  Espacio - Freno",
            "",
            "COMBATE:",
            "  Mouse - Apuntar torreta y cañón",
            "  Clic izquierdo - Disparar",
            "  1 - Proyectil ligero",
            "  2 - Proyectil pesado",
            "",
            "OTROS:",
            "  P - Salir del juego"
        };
        
        foreach (var line in controls)
        {
            var color = line.EndsWith(":") ? Color.Cyan : Color.White;
            _spriteBatch.DrawString(_menuFont, line, new Vector2(leftMargin, y), color);
            y += lineHeight;
        }
        
        // Mensaje de cierre
        var hint = "Presiona H para cerrar";
        var hintSize = _menuFont.MeasureString(hint);
        var hintPos = new Vector2(panelX + (panelW - hintSize.X) / 2, panelY + panelH - 40);
        _spriteBatch.DrawString(_menuFont, hint, hintPos, Color.Gray);
    }
}