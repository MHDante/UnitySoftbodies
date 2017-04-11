using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
public class Manipulator : MonoBehaviour {

    GameObject IndicatorObj;

    List<int> SelectedIndices = new List<int>();
    List<Vector3> offsets=new List<Vector3>();
    Plane transformPlane;

    public float selectRadius = .2f;
    public IDeformer deformer;

    // Use this for initialization
    void Start () {
        
        IndicatorObj = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        IndicatorObj.GetComponent<MeshRenderer>().material.color = Color.red;
        Destroy(IndicatorObj.GetComponent<Collider>());
        deformer = GetComponent<IDeformer>();

    }
	
	// Update is called once per frame
	void Update () {

        HandleInput();
        UpdateIndicator();
    }


    void UpdateIndicator()
    {

        IndicatorObj.transform.localScale = new Vector3(selectRadius, 5, selectRadius);
        IndicatorObj.transform.up = Camera.main.transform.position - IndicatorObj.transform.position;
    }

    void HandleInput()
    {
        if (!Input.GetMouseButton(0)) HoverVert();
        if (Input.GetMouseButtonDown(0)) InitPick();
        if (Input.GetMouseButton(0)) TransformVert();

    }

    void TransformVert()
    {
        var worldVerts = deformer.WorldVertices;
        var mouseRay = Camera.main.ScreenPointToRay(Input.mousePosition);
        float dist;

        transformPlane.Raycast(mouseRay, out dist);
        var offset = Camera.main.transform.position + dist * mouseRay.direction ;
        for (var i = 0; i < SelectedIndices.Count; i++)
        {
            int selectedIndex = SelectedIndices[i];
            var targetPosition = offsets[i] + offset;
            deformer.SetWorldVertexPosition(selectedIndex, targetPosition);
        }
        IndicatorObj.transform.position = offset;
    }

    void InitPick()
    {
        transformPlane = new Plane(Camera.main.transform.forward, IndicatorObj.transform.position);
        var mouseRay = Camera.main.ScreenPointToRay(Input.mousePosition);
        float dist;
        transformPlane.Raycast(mouseRay, out dist);
        IndicatorObj.GetComponent<MeshRenderer>().material.color = Color.yellow;
    }


    void HoverVert()
    {
        SelectedIndices.Clear();
        Vector3 sum = Vector3.zero;
        var mouseRay = Camera.main.ScreenPointToRay(Input.mousePosition);
        var vertices = deformer.WorldVertices;
        for (int i = 0; i < vertices.Length; i++)
        {
            var v = vertices[i];
            if (v.DistanceToLine(mouseRay) < selectRadius)
            {
                SelectedIndices.Add(i);
                sum += v;
            }
        }


        if (SelectedIndices.Count > 0)
        {
            IndicatorObj.transform.position = sum/ SelectedIndices.Count;
            IndicatorObj.SetActive(true);
            offsets = SelectedIndices.Select(i => vertices[i] - IndicatorObj.transform.position).ToList();
        }
        else
        {
            IndicatorObj.SetActive(false);
        }



    }

}
