// GENERATED AUTOMATICALLY FROM 'Assets/Controls.inputactions'

using System;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.Experimental.Input;


[Serializable]
public class Grabber : InputActionAssetReference
{
    public Grabber()
    {
    }
    public Grabber(InputActionAsset asset)
        : base(asset)
    {
    }
    private bool m_Initialized;
    private void Initialize()
    {
        // Main
        m_Main = asset.GetActionMap("Main");
        m_Main_VRGrabL = m_Main.GetAction("VRGrabL");
        if (m_MainVRGrabLActionStarted != null)
            m_Main_VRGrabL.started += m_MainVRGrabLActionStarted.Invoke;
        if (m_MainVRGrabLActionPerformed != null)
            m_Main_VRGrabL.performed += m_MainVRGrabLActionPerformed.Invoke;
        if (m_MainVRGrabLActionCancelled != null)
            m_Main_VRGrabL.cancelled += m_MainVRGrabLActionCancelled.Invoke;
        m_Initialized = true;
    }
    private void Uninitialize()
    {
        if (m_MainActionsCallbackInterface != null)
        {
            Main.SetCallbacks(null);
        }
        m_Main = null;
        m_Main_VRGrabL = null;
        if (m_MainVRGrabLActionStarted != null)
            m_Main_VRGrabL.started -= m_MainVRGrabLActionStarted.Invoke;
        if (m_MainVRGrabLActionPerformed != null)
            m_Main_VRGrabL.performed -= m_MainVRGrabLActionPerformed.Invoke;
        if (m_MainVRGrabLActionCancelled != null)
            m_Main_VRGrabL.cancelled -= m_MainVRGrabLActionCancelled.Invoke;
        m_Initialized = false;
    }
    public void SetAsset(InputActionAsset newAsset)
    {
        if (newAsset == asset) return;
        var MainCallbacks = m_MainActionsCallbackInterface;
        if (m_Initialized) Uninitialize();
        asset = newAsset;
        Main.SetCallbacks(MainCallbacks);
    }
    public override void MakePrivateCopyOfActions()
    {
        SetAsset(ScriptableObject.Instantiate(asset));
    }
    // Main
    private InputActionMap m_Main;
    private IMainActions m_MainActionsCallbackInterface;
    private InputAction m_Main_VRGrabL;
    [SerializeField] private ActionEvent m_MainVRGrabLActionStarted;
    [SerializeField] private ActionEvent m_MainVRGrabLActionPerformed;
    [SerializeField] private ActionEvent m_MainVRGrabLActionCancelled;
    public struct MainActions
    {
        private Grabber m_Wrapper;
        public MainActions(Grabber wrapper) { m_Wrapper = wrapper; }
        public InputAction @VRGrabL { get { return m_Wrapper.m_Main_VRGrabL; } }
        public ActionEvent VRGrabLStarted { get { return m_Wrapper.m_MainVRGrabLActionStarted; } }
        public ActionEvent VRGrabLPerformed { get { return m_Wrapper.m_MainVRGrabLActionPerformed; } }
        public ActionEvent VRGrabLCancelled { get { return m_Wrapper.m_MainVRGrabLActionCancelled; } }
        public InputActionMap Get() { return m_Wrapper.m_Main; }
        public void Enable() { Get().Enable(); }
        public void Disable() { Get().Disable(); }
        public bool enabled { get { return Get().enabled; } }
        public InputActionMap Clone() { return Get().Clone(); }
        public static implicit operator InputActionMap(MainActions set) { return set.Get(); }
        public void SetCallbacks(IMainActions instance)
        {
            if (m_Wrapper.m_MainActionsCallbackInterface != null)
            {
                VRGrabL.started -= m_Wrapper.m_MainActionsCallbackInterface.OnVRGrabL;
                VRGrabL.performed -= m_Wrapper.m_MainActionsCallbackInterface.OnVRGrabL;
                VRGrabL.cancelled -= m_Wrapper.m_MainActionsCallbackInterface.OnVRGrabL;
            }
            m_Wrapper.m_MainActionsCallbackInterface = instance;
            if (instance != null)
            {
                VRGrabL.started += instance.OnVRGrabL;
                VRGrabL.performed += instance.OnVRGrabL;
                VRGrabL.cancelled += instance.OnVRGrabL;
            }
        }
    }
    public MainActions @Main
    {
        get
        {
            if (!m_Initialized) Initialize();
            return new MainActions(this);
        }
    }
    [Serializable]
    public class ActionEvent : UnityEvent<InputAction.CallbackContext>
    {
    }
}
public interface IMainActions
{
    void OnVRGrabL(InputAction.CallbackContext context);
}
