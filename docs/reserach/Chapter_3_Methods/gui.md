If you ever notice Streamlit not firing on_change as expected, the classic robust pattern is:

store st.session_state["delan_preset_prev"]

on each run, detect if preset != prev: apply_preset_to_widgets(...) ; update prev

But since yours already uses on_change + session keys, you likely won’t need this.