using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class NameDisplay : MonoBehaviour {

    public string name;
    public Color colour;
    private GUIStyle currentStyle = null;
    // Use this for initialization

    private void InitStyles()
    {
        if (currentStyle == null)
        {
            currentStyle = new GUIStyle(GUI.skin.box);
            currentStyle.normal.background = MakeTex(2, 2, colour);
        }
    }

    private Texture2D MakeTex(int width, int height, Color col)
    {
        Color[] pix = new Color[width * height];
        for (int i = 0; i < pix.Length; ++i)
        {
            pix[i] = col;
        }
        Texture2D result = new Texture2D(width, height);
        result.SetPixels(pix);
        result.Apply();
        return result;
    }

    void OnGUI()
    {
        if (colour != null)
        {
            InitStyles();
        }
        Vector3 position = Camera.main.WorldToScreenPoint(gameObject.transform.position);
        var textSize = GUI.skin.label.CalcSize(new GUIContent(name));
        GUI.Box(new Rect(position.x - (textSize.x/2), Screen.height - position.y + 10, textSize.x*1.1f, textSize.y*1.1f), name, currentStyle);
        
        
        //GUI.Label(new Rect(position.x-25, Screen.height - position.y, textSize.x, textSize.y), name);
    }

    // Update is called once per frame

}
